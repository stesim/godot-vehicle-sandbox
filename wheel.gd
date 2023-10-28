class_name Wheel
extends RayCast3D


@export var is_driven := false

@export var is_steering := false

@export var has_handbrake := false

@export var visuals : Node3D

@export var radius := 0.4

@export var inertia := 1.0

@export var suspension : Suspension

@export var tire : Tire

@export var enforce_visual_bottom_out := true

@export_subgroup("Input")

@export var drive_torque := 0.0

@export var steering_angle : float :
	get:
		return rotation.y
	set(value):
		rotation.y = value

@export var brake_torque := 0.0

@export var drivetrain_inertia := 0.0


var _wheel_load := 0.0

var _contact_velocity := Vector3.ZERO

var _angular_velocity := 0.0

var _slip := Vector2.ZERO

var _torque_feedback := 0.0


func _ready() -> void:
	target_position.y = -(suspension.rest_length + radius)


func get_angular_velocity() -> float:
	return _angular_velocity


func get_rpm() -> float:
	return _angular_velocity / TAU * 60.0


func get_slip_velocity() -> Vector2:
	return _slip


func get_contact_velocity() -> Vector3:
	return _contact_velocity


func get_wheel_load() -> float:
	return _wheel_load


func get_torque_feedback() -> float:
	return _torque_feedback


func get_effective_inertia() -> float:
	return inertia + drivetrain_inertia


func is_bottoming_out() -> bool:
	if not is_colliding():
		return false
	var contact_distance := (global_position - get_collision_point()).dot(global_transform.basis.y)
	return contact_distance < radius


func apply_suspension_force(vehicle_state : PhysicsDirectBodyState3D) -> void:
	if not is_colliding():
		_contact_velocity = Vector3.ZERO
		_wheel_load = 0.0
		return

	var contact_point := get_collision_point()
	_contact_velocity = vehicle_state.get_velocity_at_local_position(contact_point - vehicle_state.transform.origin)

	var suspension_force := suspension.calculate_force(vehicle_state, self)

	var force_vector := suspension_force * global_transform.basis.y
	var force_position := contact_point - vehicle_state.transform.origin
	vehicle_state.apply_force(force_vector, force_position)

	# TODO: consider velocity of collider
	_wheel_load = suspension_force


func apply_bottom_out_force(vehicle_state : PhysicsDirectBodyState3D, virtual_mass : float) -> void:
	var up := global_transform.basis.y
	var excess_linear_velocity := maxf(0.0, -_contact_velocity.dot(up))
	var force := virtual_mass * excess_linear_velocity / vehicle_state.step * up
	var force_position := get_collision_point() - vehicle_state.transform.origin
	vehicle_state.apply_force(force, force_position)


func apply_drive_forces(vehicle_state : PhysicsDirectBodyState3D) -> void:
	_torque_feedback = 0.0

	var applied_brake_torque := 0.0

	_angular_velocity += vehicle_state.step * drive_torque / get_effective_inertia()
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

	var traction := tire.calculate_traction(vehicle_state, self, applied_brake_torque)

	var feedback_torque := -traction.x * radius
	_angular_velocity += vehicle_state.step * feedback_torque / get_effective_inertia()
	_torque_feedback += feedback_torque

	var traction_force := traction.x * forward + traction.y * right
	var force_position := get_collision_point() - vehicle_state.transform.origin
	vehicle_state.apply_force(traction_force, force_position)

	_slip = _calculate_slip(forward, right)


func apply_rolling_resistance(vehicle_state : PhysicsDirectBodyState3D) -> void:
	var forward := -vehicle_state.transform.basis.z
	var speed := vehicle_state.linear_velocity.dot(forward)
	var force_limit := -speed / (vehicle_state.inverse_mass * vehicle_state.step)
	var rolling_resistance := -signf(speed) * tire.rolling_resistance_coefficient * _wheel_load
	if rolling_resistance / force_limit > 1.0:
		rolling_resistance = force_limit
	vehicle_state.apply_central_force(rolling_resistance * forward)


func _apply_brake_torque(torque : float, delta : float) -> float:
	var effective_inertia := get_effective_inertia()
	var brake_torque_limit := -_angular_velocity * effective_inertia / delta
	var ratio := torque / brake_torque_limit
	if ratio < 0.0:
		torque = 0.0
	elif ratio > 1.0:
		torque = brake_torque_limit
	_angular_velocity += delta * torque / effective_inertia
	_torque_feedback += torque
	return torque


func _calculate_slip(forward : Vector3, right : Vector3) -> Vector2:
	var rotation_velocity := radius * _angular_velocity * forward
	var relative_velocity := rotation_velocity - _contact_velocity
	var slip := Vector2(
		relative_velocity.dot(forward),
		relative_velocity.dot(right)
	)
	return slip


func _physics_process(delta : float) -> void:
	_update_visuals(delta)


func _update_visuals(delta : float) -> void:
	if is_colliding():
		visuals.global_position = get_collision_point()
	else:
		visuals.position = target_position

	visuals.position.y += radius

	# avoid visual glitches when suspension has been compressed beyond the bottom-out position
	if enforce_visual_bottom_out and visuals.position.y > 0.0:
		visuals.position.y = 0.0

	visuals.rotate(Vector3.LEFT, _angular_velocity * delta)
