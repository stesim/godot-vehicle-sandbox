class_name Tire
extends Resource


@export_range(0.0, 0.1, 0.001, "or_greater") var rolling_resistance_coefficient := 0.01

@export_range(0.0, 2.0, 0.01, "or_greater") var friction_limit := 1.0

@export_range(0.0, 1.0) var magic_tweak_factor := 0.5

@export var longitudinal_traction_curve : Curve

@export var longitudinal_traction_curve_scale := Vector2.ONE

@export var lateral_traction_curve : Curve

@export var lateral_traction_curve_scale := Vector2.ONE


func calculate_traction(vehicle_state : PhysicsDirectBodyState3D, wheel : Wheel, available_brake_torque : float, applied_brake_torque : float) -> Vector2:
	var wheel_right := wheel.global_transform.basis.x
	var surface_normal := wheel.get_contact_normal()
	var surface_forward := surface_normal.cross(wheel_right).normalized()
	var surface_right := surface_forward.cross(surface_normal).normalized()

	var slip := wheel.get_slip_velocity()
	var contact_velocity := wheel.get_contact_velocity()
	var local_contact_velocity := Vector2(
		contact_velocity.dot(surface_forward),
		contact_velocity.dot(surface_right),
	)
	var grip := _calculate_grip(slip, local_contact_velocity)
	var tire_load := wheel.get_wheel_load()
	var wheel_normal := wheel_right.cross(surface_forward).normalized()
	var camber_factor := absf(surface_normal.dot(wheel_normal))
	var traction := camber_factor * tire_load * grip

	var traction_limits := _calculate_traction_limits(vehicle_state, wheel, slip, surface_forward, surface_right)
	traction_limits *= camber_factor

	# NOTE: brake must be considered, since it will potentially counteract the traction force
	traction_limits.x += signf(traction_limits.x) * available_brake_torque / wheel.radius
	if traction_limits.x / applied_brake_torque > 0.0:
		traction_limits.x -= applied_brake_torque / wheel.radius

	if traction.x / traction_limits.x < 0.0:
		traction.x = 0.0
	elif traction.x / traction_limits.x > 1.0:
		traction.x = traction_limits.x

	if traction.y / traction_limits.y < 0.0:
		traction.y = 0.0
	elif traction.y / traction_limits.y > 1.0:
		traction.y = traction_limits.y

	return traction


func _calculate_grip(slip : Vector2, contact_velocity : Vector2) -> Vector2:
	var slip_ratio := slip.x / maxf(0.001, contact_velocity.x)
	var slip_angle := -atan2(contact_velocity.y, absf(contact_velocity.y))

	var weight := Vector2.ONE
	if magic_tweak_factor > 0.0:
		weight = weight.slerp(slip.normalized().abs(), magic_tweak_factor)

	var grip := weight * Vector2(
		signf(slip_ratio) * _sample_curve(longitudinal_traction_curve, longitudinal_traction_curve_scale, absf(slip_ratio)),
		signf(slip_angle) * _sample_curve(lateral_traction_curve, lateral_traction_curve_scale, absf(slip_angle))
	).limit_length(friction_limit)

	return grip


func _calculate_traction_limits(vehicle_state : PhysicsDirectBodyState3D, wheel : Wheel, slip : Vector2, forward : Vector3, right : Vector3) -> Vector2:
	var force_position := wheel.get_contact_point() - vehicle_state.transform.origin

	var dv_rotation := -vehicle_state.step * wheel.radius * wheel.radius / wheel.get_effective_inertia()
	var dv_contact_x := vehicle_state.step * (vehicle_state.inverse_mass + (vehicle_state.inverse_inertia_tensor * force_position.cross(forward)).cross(force_position).dot(forward))
	var dv_contact_y := vehicle_state.step * (vehicle_state.inverse_mass + (vehicle_state.inverse_inertia_tensor * force_position.cross(right)).cross(force_position).dot(right))

	var traction_force_limits := Vector2(
		-slip.x / (dv_rotation - dv_contact_x) - vehicle_state.total_gravity.dot(forward),
		-slip.y / (-dv_contact_y) - vehicle_state.total_gravity.dot(right),
	)
	return traction_force_limits


func _sample_curve(curve : Curve, coordinate_scale : Vector2, point : float) -> float:
	var scaled_point := clampf(point / coordinate_scale.x, 0.0, 1.0)
	return curve.sample_baked(scaled_point) * coordinate_scale.y
