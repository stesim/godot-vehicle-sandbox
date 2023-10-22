class_name Vehicle
extends RigidBody3D


@export var max_engine_torque := 4.0 * 3.0 * 500.0

@export var max_steering_angle := deg_to_rad(30.0)

@export var wheels : Array[Wheel] = []

@export var input_speed := 6.0


var _engine_input := 0.0

var _steering_input := 0.0


func _process(delta : float) -> void:
	_update_inputs(delta)


func _integrate_forces(state : PhysicsDirectBodyState3D) -> void:
	_apply_wheel_inputs(state.step)
	_apply_suspension_forces(state)
	_apply_tire_forces(state)


func _apply_wheel_inputs(delta : float) -> void:
	for wheel in wheels:
		wheel.apply_inputs(delta)


func _apply_suspension_forces(state : PhysicsDirectBodyState3D) -> void:
	for wheel in wheels:
		wheel.apply_suspension_force(state)

	for wheel in wheels:
		if wheel.is_bottoming_out():
			# NOTE: assumes uniform mass distribution, but appears to work reasonably well
			# TODO: more accurate solution?
			var virtual_mass := mass / wheels.size()
			wheel.apply_bottom_out_impulse(state, virtual_mass)


func _apply_tire_forces(state : PhysicsDirectBodyState3D) -> void:
	for wheel in wheels:
		var virtual_mass := mass / wheels.size()
		wheel.apply_tire_forces(state, virtual_mass)


func _update_inputs(delta : float) -> void:
	_engine_input = move_toward(_engine_input, Input.get_axis(&"brake", &"accelerate"), delta * input_speed)
	var engine_torque := _engine_input * max_engine_torque

	var num_driven_wheels := 0
	for wheel in wheels:
		if wheel.is_driven:
			num_driven_wheels += 1

	for wheel in wheels:
		if wheel.is_driven:
			var wheel_torque := engine_torque / num_driven_wheels
			wheel.drive_torque = wheel_torque

	_steering_input = move_toward(_steering_input, Input.get_axis(&"steer_right", &"steer_left"), delta * input_speed)
	_apply_steering_input()


func _apply_steering_input() -> void:
	var steering_angle := _steering_input * max_steering_angle
	if is_zero_approx(steering_angle):
		for wheel in wheels:
			if wheel.is_steering:
				wheel.steering_angle = steering_angle
		return

	var turn_center := _calculate_turn_center()
	_apply_ackermann_steering(turn_center, steering_angle)


func _apply_ackermann_steering(turn_center : Vector3, steering_angle : float) -> void:
	for wheel in wheels:
		if wheel.is_steering:
			var turn_center_offset := wheel.position - turn_center
			var turn_radius := -turn_center_offset.z / tan(steering_angle)
			var ackermann_angle = atan(-turn_center_offset.z / (turn_radius + turn_center_offset.x))
			wheel.steering_angle = ackermann_angle


func _calculate_turn_center() -> Vector3:
	var turn_center := Vector3.ZERO
	var num_non_steering_wheels := 0
	for wheel in wheels:
		if not wheel.is_steering:
			turn_center += wheel.position
			num_non_steering_wheels += 1
	turn_center /= num_non_steering_wheels
	return turn_center
