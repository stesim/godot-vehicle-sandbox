class_name Wheel
extends Node3D


@export var visuals : Node3D

@export var has_handbrake := false

@export_range(0.0, 1.0, 0.01, "or_greater") var radius := 0.4

@export_range(0.0, 1.0, 0.01, "or_greater") var width := 0.3

@export_range(0.0, 2.0, 0.01, "or_greater") var inertia := 1.0

@export_range(0.0, 1.0, 0.01, "or_greater") var traction_control_slip_ratio_threshold := 0.2

@export var suspension : Suspension

@export var tire : Tire

@export var brake : Brake

@export var enforce_visual_bottom_out := true

@export_subgroup("Input")

@export_range(0.0, 4.0, 0.01, "or_greater") var drivetrain_inertia := 0.0

@export_range(0.0, 20000.0, 0.01, "or_greater") var drive_torque := 0.0

@export_range(-90.0, 90.0, 0.01, "radians") var steering_angle : float :
	get:
		return rotation.y
	set(value):
		rotation.y = value

@export_range(0.0, 1.0) var brake_input := 0.0

@export_range(0.0, 1.0) var handbrake_input := 0.0

@export var traction_control_enabled := true


var _is_in_contact := false

var _contact_point := Vector3.ZERO

var _contact_normal := Vector3.ZERO

var _suspension_length := 0.0

var _wheel_load := 0.0

var _contact_velocity := Vector3.ZERO

var _angular_velocity := 0.0

var _brake_torque := 0.0

var _slip := Vector2.ZERO

var _torque_feedback := 0.0


@onready var _shape_cast_query := _create_shape_cast_query()


func _ready() -> void:
	_suspension_length = suspension.rest_length


func get_angular_velocity() -> float:
	return _angular_velocity


func get_rpm() -> float:
	return _angular_velocity / TAU * 60.0


func get_slip_velocity() -> Vector2:
	return _slip


func get_contact_point() -> Vector3:
	return _contact_point


func get_contact_normal() -> Vector3:
	return _contact_normal


func get_contact_velocity() -> Vector3:
	return _contact_velocity


func get_suspension_length() -> float:
	return _suspension_length


func get_wheel_load() -> float:
	return _wheel_load


func get_torque_feedback() -> float:
	return _torque_feedback


func get_effective_inertia() -> float:
	return inertia + drivetrain_inertia


func is_in_contact() -> bool:
	return _is_in_contact


func is_bottoming_out() -> bool:
	return _suspension_length < 0.0


func apply_suspension_force(vehicle_state : PhysicsDirectBodyState3D) -> void:
	_find_contact_point(vehicle_state.get_space_state())

	if not _is_in_contact:
		_suspension_length = suspension.rest_length
		_wheel_load = 0.0
		return

	var up := global_transform.basis.y
	var new_suspension_length := (global_position - _contact_point).dot(up) - radius
	var suspension_velocity := (new_suspension_length - _suspension_length) / vehicle_state.step
	_suspension_length = new_suspension_length
	_contact_velocity = vehicle_state.get_velocity_at_local_position(_contact_point - vehicle_state.transform.origin)

	var suspension_force := suspension.calculate_force(vehicle_state, new_suspension_length, suspension_velocity)

	var force_vector := suspension_force * global_transform.basis.y
	var force_position := global_position - vehicle_state.transform.origin
	vehicle_state.apply_force(force_vector, force_position)

	_wheel_load = suspension_force


func apply_bottom_out_force(vehicle_state : PhysicsDirectBodyState3D, virtual_mass : float) -> void:
	var up := global_transform.basis.y
	var excess_linear_velocity := maxf(0.0, -_contact_velocity.dot(up))
	var force := virtual_mass * excess_linear_velocity / vehicle_state.step * up
	var force_position := global_position - vehicle_state.transform.origin
	vehicle_state.apply_force(force, force_position)


func apply_drive_forces(vehicle_state : PhysicsDirectBodyState3D) -> void:
	_torque_feedback = 0.0
	_brake_torque = brake_input * brake.max_torque
	if has_handbrake:
		_brake_torque = maxf(_brake_torque, handbrake_input * brake.max_handbrake_torque)

	if traction_control_enabled:
		_apply_traction_control(vehicle_state)

	var applied_brake_torque := 0.0

	_angular_velocity += vehicle_state.step * drive_torque / get_effective_inertia()
	var drive_brake_torque := -signf(_angular_velocity) * _brake_torque - applied_brake_torque
	applied_brake_torque += _apply_brake_torque(drive_brake_torque, vehicle_state.step)

	if is_in_contact():
		_apply_tire_forces(vehicle_state, applied_brake_torque)
		var traction_brake_torque := -signf(_angular_velocity) * _brake_torque - applied_brake_torque
		applied_brake_torque += _apply_brake_torque(traction_brake_torque, vehicle_state.step)
	else:
		_slip = Vector2.ZERO


func apply_rolling_resistance(vehicle_state : PhysicsDirectBodyState3D) -> void:
	var forward := -vehicle_state.transform.basis.z
	var speed := vehicle_state.linear_velocity.dot(forward)
	var force_limit := -speed / (vehicle_state.inverse_mass * vehicle_state.step)
	var rolling_resistance := -signf(speed) * tire.rolling_resistance_coefficient * _wheel_load
	if rolling_resistance / force_limit > 1.0:
		rolling_resistance = force_limit
	vehicle_state.apply_central_force(rolling_resistance * forward)


func _apply_tire_forces(vehicle_state : PhysicsDirectBodyState3D, applied_brake_torque : float) -> void:
	var forward := _contact_normal.cross(global_transform.basis.x).normalized()
	var right := forward.cross(_contact_normal).normalized()

	_slip = _calculate_slip(forward, right)

	var traction := tire.calculate_traction(vehicle_state, self, _brake_torque, applied_brake_torque)

	var feedback_torque := -traction.x * radius
	_angular_velocity += vehicle_state.step * feedback_torque / get_effective_inertia()
	_torque_feedback += feedback_torque

	var traction_force := traction.x * forward + traction.y * right
	var force_position := _contact_point - vehicle_state.transform.origin
	vehicle_state.apply_force(traction_force, force_position)

	_slip = _calculate_slip(forward, right)


func _apply_traction_control(vehicle_state : PhysicsDirectBodyState3D) -> void:
	var forward := _contact_normal.cross(global_transform.basis.x).normalized()
	var forward_velocity := _contact_velocity.dot(forward)
	if absf(forward_velocity) < 0.1:
		forward_velocity = signf(forward_velocity) * 0.1
	var angular_velocity_threshold := (traction_control_slip_ratio_threshold + 1.0) * forward_velocity / radius
	var excess_angular_velocity := _angular_velocity - angular_velocity_threshold
	if signf(forward_velocity) * excess_angular_velocity > 0.0:
		var required_torque := -excess_angular_velocity / vehicle_state.step * get_effective_inertia()
		var resistive_torque := absf(required_torque)
		_brake_torque = minf(_brake_torque + resistive_torque, brake.max_torque)


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


func _find_contact_point(space_state : PhysicsDirectSpaceState3D) -> void:
	_is_in_contact = false
	_contact_point = Vector3.ZERO
	_contact_normal = Vector3.ZERO

	_shape_cast_query.transform = Transform3D().rotated(Vector3.FORWARD, 0.5 * PI).translated(global_position)
	_shape_cast_query.motion = -suspension.rest_length * global_transform.basis.y
	var results := space_state.cast_motion(_shape_cast_query)

	var motion_fraction := results[1]
	if not is_equal_approx(motion_fraction, 1.0):
		_shape_cast_query.transform.origin += motion_fraction * _shape_cast_query.motion

	var intersection := space_state.get_rest_info(_shape_cast_query)
	if intersection.is_empty():
		return

	_is_in_contact = true
	_contact_point = intersection.point
	_contact_normal = intersection.normal


func _process(delta : float) -> void:
	_update_visuals(delta)


func _update_visuals(delta : float) -> void:
	visuals.position.y = -_suspension_length

	# avoid visual glitches when suspension has been compressed beyond the bottom-out position
	if enforce_visual_bottom_out and visuals.position.y > 0.0:
		visuals.position.y = 0.0

	visuals.rotate(Vector3.LEFT, _angular_velocity * delta)


func _create_shape_cast_query() -> PhysicsShapeQueryParameters3D:
	var vehicle := get_parent()
	while vehicle != null and not vehicle is Vehicle:
		vehicle = vehicle.get_parent()

	var cylinder := CylinderShape3D.new()
	cylinder.height = width
	cylinder.radius = radius

	var query := PhysicsShapeQueryParameters3D.new()
	if vehicle != null:
		query.exclude = [vehicle.get_rid()]
	query.shape = cylinder
	return query
