class_name Axle
extends Node3D


@export var is_steering := false

@export var is_driven := false

@export var differential : Differential

@export var anti_roll_bar : AntiRollBar


@export_subgroup("Input")

@export_range(0.0, 20000.0, 0.01, "or_greater") var torque_input := 0.0

@export_range(-90.0, 90.0, 0.01, "radians") var steering_angle := 0.0

@export var turn_center := Vector3.ZERO


var _wheels : Array[Wheel] = []


func get_wheels() -> Array[Wheel]:
	return _wheels


func update(vehicle_state : PhysicsDirectBodyState3D, virtual_mass : float) -> void:
	_apply_suspension_forces(vehicle_state, virtual_mass)
	if is_steering:
		_apply_steering_input()
	if is_driven:
		_apply_drive_input()
	_apply_tire_forces(vehicle_state)


func get_torque_feedback() -> float:
	var torque := 0.0
	for wheel in _wheels:
		torque += wheel.get_torque_feedback()
	torque /= _wheels.size()
	return torque


func get_angular_velocity_feedback() -> float:
	var angular_velocity := 0.0
	for wheel in _wheels:
		angular_velocity += wheel.get_angular_velocity()
	angular_velocity /= _wheels.size()
	return angular_velocity


func _init() -> void:
	child_entered_tree.connect(_on_child_entered)
	child_exiting_tree.connect(_on_child_exiting)


func _apply_suspension_forces(vehicle_state : PhysicsDirectBodyState3D, virtual_mass : float) -> void:
	for wheel in _wheels:
		wheel.apply_suspension_force(vehicle_state)

	if anti_roll_bar != null:
		_apply_anti_roll_bar_forces(vehicle_state)

	var virtual_mass_per_wheel := virtual_mass / _wheels.size()
	for wheel in _wheels:
		if wheel.is_bottoming_out():
			wheel.apply_bottom_out_force(vehicle_state, virtual_mass_per_wheel)


func _apply_steering_input() -> void:
	for wheel in _wheels:
		var turn_center_offset := position + wheel.position - turn_center
		var turn_radius := -turn_center_offset.z / tan(steering_angle)
		var ackermann_angle = atan(-turn_center_offset.z / (turn_radius + turn_center_offset.x))
		wheel.steering_angle = ackermann_angle


func _apply_drive_input() -> void:
	if differential != null and _wheels.size() == 2:
		differential.torque_input = torque_input
		differential.velocity_feedback_1 = _wheels[0].get_angular_velocity()
		differential.velocity_feedback_2 = _wheels[1].get_angular_velocity()
		differential.torque_feedback_1 = _wheels[0].get_torque_feedback()
		differential.torque_feedback_2 = _wheels[1].get_torque_feedback()
		differential.update()

		_wheels[0].drive_torque = differential.get_torque_output_1()
		_wheels[1].drive_torque = differential.get_torque_output_2()
	else:
		# FIXME: this behaves like an open differential instead of a fixed axle
		for wheel in _wheels:
			wheel.drive_torque = torque_input / _wheels.size()


func _apply_tire_forces(vehicle_state : PhysicsDirectBodyState3D) -> void:
	for wheel in _wheels:
		wheel.apply_drive_forces(vehicle_state)
		wheel.apply_rolling_resistance(vehicle_state)


func _apply_anti_roll_bar_forces(vehicle_state : PhysicsDirectBodyState3D) -> void:
	var average_travel := 0.0
	for wheel in _wheels:
		average_travel += wheel.get_suspension_length()
	average_travel /= _wheels.size()

	anti_roll_bar.average_travel = average_travel
	for wheel in _wheels:
		var force := anti_roll_bar.calculate_force(wheel.get_suspension_length())
		var up := wheel.global_transform.basis.y
		var force_position := wheel.global_position - vehicle_state.transform.origin
		vehicle_state.apply_force(force * up, force_position)


func _on_child_entered(child : Node) -> void:
	if child is Wheel:
		_wheels.push_back(child)


func _on_child_exiting(child : Node) -> void:
	if child is Wheel:
		_wheels.erase(child)
