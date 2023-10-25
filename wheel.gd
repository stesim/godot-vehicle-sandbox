class_name Wheel
extends RayCast3D


@export var visuals : Node3D

@export var radius := 0.4

@export var inertia := 1.0

@export_subgroup("Suspension")

@export var rest_length := 0.3

@export var stiffness := 40000.0

@export var stiffness_curve : Curve

@export var damping := 4000.0

@export_subgroup("Tire")

@export var peak_longitudinal_slip := 0.25

@export var peak_lateral_slip := 0.25

@export var longitudinal_traction_curve : Curve

@export var lateral_traction_curve : Curve

@export_subgroup("Input")

@export var is_driven := false

@export var drive_torque := 0.0

@export var is_steering := false

@export var steering_angle : float :
	get:
		return rotation.y
	set(value):
		rotation.y = value

@export var brake_torque := 0.0

@export var has_handbrake := false



var _spring_displacement := 0.0

var _suspension_force := 0.0

var _contact_velocity := Vector3.ZERO

var _angular_velocity := 0.0

var _slip := Vector2.ZERO


func _ready() -> void:
	var total_length := rest_length + radius
	target_position.y = -total_length


func get_suspension_force() -> float:
	return _suspension_force


func is_bottoming_out() -> bool:
	return -_spring_displacement > rest_length


func get_angular_velocity() -> float:
	return _angular_velocity


func get_rpm() -> float:
	return _angular_velocity / TAU * 60.0


func get_slip_velocity() -> Vector2:
	return _slip


func apply_suspension_force(vehicle_state : PhysicsDirectBodyState3D) -> void:
	if not is_colliding():
		_suspension_force = 0.0
		return

	var up := global_transform.basis.y
	var contact_point := get_collision_point()
	var contact_distance := (global_position - contact_point).dot(up)
	var previous_displacement := _spring_displacement
	_spring_displacement = contact_distance - (rest_length + radius)
	var displacement_velocity = (_spring_displacement - previous_displacement) / vehicle_state.step

	var effective_stiffness := stiffness
	if stiffness_curve != null:
		effective_stiffness *= stiffness_curve.sample_baked(-_spring_displacement / rest_length)
	_suspension_force = -effective_stiffness * _spring_displacement - damping * displacement_velocity

	var force_vector := _suspension_force * up
	var force_position := contact_point - vehicle_state.transform.origin
	vehicle_state.apply_force(force_vector, force_position)

	# TODO: consider velocity of collider
	_contact_velocity = vehicle_state.get_velocity_at_local_position(contact_point - vehicle_state.transform.origin)


func apply_bottom_out_impulse(vehicle_state : PhysicsDirectBodyState3D, virtual_mass : float) -> void:
	var up := global_transform.basis.y
	var wheel_velocity_along_spring := _contact_velocity.dot(up)
	if wheel_velocity_along_spring < 0.0:
		# TODO: compute impulse so that it compensates for the invalid position and angular velocity
		#       instead of only stopping the linear velocity
		var bottom_out_impulse := -virtual_mass * wheel_velocity_along_spring * up
		var impulse_position := get_collision_point() - vehicle_state.transform.origin
		vehicle_state.apply_force(bottom_out_impulse / vehicle_state.step, impulse_position)


func apply_drive_forces(vehicle_state : PhysicsDirectBodyState3D) -> void:
	var applied_brake_torque := 0.0

	_angular_velocity += vehicle_state.step * drive_torque / inertia
	var drive_brake_torque := -signf(_angular_velocity) * brake_torque - applied_brake_torque
	applied_brake_torque += _apply_brake_torque(drive_brake_torque, vehicle_state.step)

	if is_colliding():
		_apply_tire_forces(vehicle_state, applied_brake_torque)
		var traction_brake_torque := -signf(_angular_velocity) * brake_torque - applied_brake_torque
		applied_brake_torque += _apply_brake_torque(traction_brake_torque, vehicle_state.step)
	else:
		_slip = Vector2.ZERO


func _apply_tire_forces(vehicle_state : PhysicsDirectBodyState3D, applied_brake_torque : float) -> void:
	var contact_normal := get_collision_normal()
	var forward := contact_normal.cross(global_transform.basis.x).normalized()
	var right := forward.cross(contact_normal).normalized()

	_slip = _calculate_slip(forward, right)
	var grip := _calculate_grip(_slip)
	var tire_load := _suspension_force
	var traction := tire_load * grip

	var traction_limits := _calculate_traction_limits(vehicle_state, _slip, forward, right)

	# NOTE: brake must be considered, since it will potentially counteract the traction force
	traction_limits.x += signf(traction_limits.x) * brake_torque / radius
	if traction_limits.x / applied_brake_torque > 0.0:
		traction_limits.x -= applied_brake_torque / radius

	if traction.x / traction_limits.x < 0.0:
		traction.x = 0.0
	elif traction.x / traction_limits.x > 1.0:
		traction.x = traction_limits.x

	#if traction.y / traction_limits.y < 0.0:
	#	traction.y = 0.0
	#elif traction.y / traction_limits.y > 1.0:
	#	traction.y = traction_limits.y

	var feedback_torque := -traction.x * radius
	_angular_velocity += vehicle_state.step * feedback_torque / inertia

	var traction_force := traction.x * forward + traction.y * right
	var force_position := get_collision_point() - vehicle_state.transform.origin
	vehicle_state.apply_force(traction_force, force_position)

	_slip = _calculate_slip(forward, right)


func _calculate_traction_limits(vehicle_state : PhysicsDirectBodyState3D, slip : Vector2, forward : Vector3, right : Vector3) -> Vector2:
	var force_position := get_collision_point() - vehicle_state.transform.origin

	var dv_rotation := -vehicle_state.step * radius * radius / inertia
	var dv_contact_x := vehicle_state.step * (vehicle_state.inverse_mass + (vehicle_state.inverse_inertia_tensor * forward.cross(force_position)).cross(force_position).dot(forward))
	var dv_contact_y := vehicle_state.step * (vehicle_state.inverse_mass + (vehicle_state.inverse_inertia_tensor * right.cross(force_position)).cross(force_position).dot(right))

	var traction_force_limits := Vector2(
		-slip.x / (dv_rotation - dv_contact_x) - vehicle_state.total_gravity.dot(forward),
		-slip.y / (-dv_contact_y) - vehicle_state.total_gravity.dot(right),
	)
	return traction_force_limits


func _apply_brake_torque(torque : float, delta : float) -> float:
	var brake_torque_limit := -_angular_velocity * inertia / delta
	var ratio := torque / brake_torque_limit
	if ratio < 0.0:
		torque = 0.0
	elif ratio > 1.0:
		torque = brake_torque_limit
	_angular_velocity += delta * torque / inertia
	return torque


func _calculate_slip(forward : Vector3, right : Vector3) -> Vector2:
	var rotation_velocity := radius * _angular_velocity * forward
	var relative_velocity := rotation_velocity - _contact_velocity
	var slip := Vector2(
		relative_velocity.dot(forward),
		relative_velocity.dot(right)
	)
	return slip


func _calculate_grip(slip : Vector2) -> Vector2:
	var normalized_slip := Vector2(
		slip.x / peak_longitudinal_slip,
		slip.y / peak_lateral_slip
	)
	if normalized_slip.is_zero_approx():
		return Vector2.ZERO
	var length := normalized_slip.length()
	var grip := Vector2(
		normalized_slip.x / length * longitudinal_traction_curve.sample_baked(length * peak_longitudinal_slip),
		normalized_slip.y / length * lateral_traction_curve.sample_baked(length * peak_lateral_slip)
	)
	return grip


func _physics_process(delta : float) -> void:
	_update_visuals(delta)


func _update_visuals(delta : float) -> void:
	if is_colliding():
		visuals.global_position = get_collision_point()
	else:
		visuals.position = target_position

	visuals.position.y += radius

	visuals.rotate(Vector3.LEFT, _angular_velocity * delta)
