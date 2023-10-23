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
		var bottom_out_impulse := -virtual_mass * wheel_velocity_along_spring * up
		var impulse_position := get_collision_point() - vehicle_state.transform.origin
		vehicle_state.apply_force(bottom_out_impulse / vehicle_state.step, impulse_position)


func apply_drive_forces(vehicle_state : PhysicsDirectBodyState3D, virtual_mass : float) -> void:
	_angular_velocity += vehicle_state.step * drive_torque / inertia
	var applied_brake_torque := _apply_brake_torque(-signf(_angular_velocity) * brake_torque, vehicle_state.step)

	if is_colliding():
		_apply_tire_forces(vehicle_state, virtual_mass)
		var traction_brake_torque := -signf(_angular_velocity) * brake_torque - applied_brake_torque
		applied_brake_torque += _apply_brake_torque(traction_brake_torque, vehicle_state.step)


func _apply_tire_forces(vehicle_state : PhysicsDirectBodyState3D, virtual_mass : float) -> void:
	var contact_normal := get_collision_normal()
	var forward := contact_normal.cross(global_transform.basis.x).normalized()
	var right := forward.cross(contact_normal).normalized()

	_slip = _calculate_slip(forward, right)
	var grip := _calculate_grip(_slip)

	var tire_load := _suspension_force
	var longitudinal_traction_force := tire_load * grip.x
	var lateral_traction_force := tire_load * grip.y

	#var lateral_traction_force_limit := virtual_mass * slip.y / vehicle_state.step
	#if not is_zero_approx(lateral_traction_force_limit) and lateral_traction_force / lateral_traction_force_limit > 1.0:
	#	lateral_traction_force = lateral_traction_force_limit

	var traction_force := longitudinal_traction_force * forward + lateral_traction_force * right
	var force_position := get_collision_point() - vehicle_state.transform.origin
	vehicle_state.apply_force(traction_force, force_position)

	_apply_traction_feedback(vehicle_state, _slip.x, virtual_mass, forward, longitudinal_traction_force)


func _apply_traction_feedback(vehicle_state : PhysicsDirectBodyState3D, slip : float, virtual_mass : float, forward : Vector3, traction_force : float) -> void:
	var contact_point_local := get_collision_point() - vehicle_state.transform.origin
	var vehicle_inertia_inv := vehicle_state.inverse_inertia_tensor

	var dv_rotation := -vehicle_state.step * radius * radius / inertia
	var dv_contact := vehicle_state.step * (1.0 / virtual_mass + (vehicle_inertia_inv * forward.cross(contact_point_local)).cross(contact_point_local).dot(forward))
	var ds_max := -slip
	var gravity := vehicle_state.total_gravity.dot(forward)

	var traction_torque := -traction_force * radius
	var traction_torque_limit := -radius * (ds_max / (dv_rotation - dv_contact) - gravity)
	if traction_torque / traction_torque_limit > 1.0:
		traction_torque = traction_torque_limit
	_angular_velocity += vehicle_state.step * traction_torque / inertia


func _apply_brake_torque(torque : float, delta : float) -> float:
	if is_zero_approx(_angular_velocity):
		return 0.0
	var brake_torque_limit := -_angular_velocity * inertia / delta
	if torque / brake_torque_limit > 1.0:
		torque = brake_torque_limit
	_angular_velocity += delta * torque / inertia
	return torque


func _calculate_slip(forward : Vector3, right : Vector3) -> Vector2:
	var rotation_velocity := radius * _angular_velocity
	var slip := Vector2(
		rotation_velocity - _contact_velocity.dot(forward),
		-_contact_velocity.dot(right)
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
