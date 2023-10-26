class_name Vehicle
extends RigidBody3D


@export var max_steering_angle := deg_to_rad(30.0)

@export var max_brake_torque := 4000.0

@export var max_handbrake_torque := 1550.0

@export var auto_brake_threshold := 0.25

@export var motor : Motor

@export var transmission : Transmission

@export var wheels : Array[Wheel] = []

@export var input_speed := 6.0

@export var motor_audio_controller : MotorAudioController

@export var wheel_audio_controller : TireAudioController


var _engine_input := 0.0

var _steering_input := 0.0

var _brake_input := 0.0

var _handbrake_input := 0.0


func _ready() -> void:
	transmission.gear = transmission.neutral_gear + 1

	#linear_damp_mode = RigidBody3D.DAMP_MODE_REPLACE
	#linear_damp = 0.0056
	#await get_tree().create_timer(1.0).timeout
	#apply_central_impulse(10000.0 * -global_transform.basis.z)


func _process(delta : float) -> void:
	_update_inputs(delta)
	_update_audio()


func _integrate_forces(state : PhysicsDirectBodyState3D) -> void:
	_apply_drive_input(state.step)
	_apply_suspension_forces(state)
	_apply_tire_forces(state)


func _unhandled_input(event : InputEvent) -> void:
	if event.is_action_pressed(&"shift_up"):
		transmission.shift_relative(+1)
	elif event.is_action_pressed(&"shift_down"):
		transmission.shift_relative(-1)


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
			wheel.apply_bottom_out_force(state, virtual_mass)


func _apply_tire_forces(state : PhysicsDirectBodyState3D) -> void:
	for wheel in wheels:
		wheel.apply_drive_forces(state)


func _update_inputs(delta : float) -> void:
	_engine_input = move_toward(_engine_input, Input.get_action_strength(&"accelerate"), delta * input_speed)
	_brake_input = move_toward(_brake_input, Input.get_action_strength(&"brake"), delta * input_speed)
	_handbrake_input = move_toward(_handbrake_input, Input.get_action_strength(&"handbrake"), delta * input_speed)
	_steering_input = move_toward(_steering_input, Input.get_axis(&"steer_right", &"steer_left"), delta * input_speed)

	_apply_steering_input()


func _apply_drive_input(delta : float) -> void:
	var num_driven_wheels := 0
	var brake_torque := 0.0
	var handbrake_torque := _handbrake_input * max_handbrake_torque

	for wheel in wheels:
		if wheel.is_driven:
			num_driven_wheels += 1

	motor.throttle = _engine_input
	brake_torque = _brake_input * max_brake_torque

	if transmission.gear == transmission.neutral_gear:
		motor.rpm_feedback = -1.0
	else:
		var gear_ratio := transmission.get_current_gear_ratio()
		var feedback_rpm := INF
		for wheel in wheels:
			if wheel.is_driven:
				feedback_rpm = minf(feedback_rpm, gear_ratio * wheel.get_rpm())
		motor.rpm_feedback = feedback_rpm

	motor.update(delta)
	transmission.torque_input = motor.get_torque_output()
	transmission.update(delta)

	var wheel_torque := transmission.get_torque_output() / num_driven_wheels if num_driven_wheels > 0 else 0.0
	for wheel in wheels:
		wheel.brake_torque = maxf(brake_torque, handbrake_torque) if wheel.has_handbrake else brake_torque
		if wheel.is_driven:
			wheel.drive_torque = wheel_torque


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


func _update_audio() -> void:
	_update_motor_audio()
	_update_wheel_audio()


func _update_motor_audio() -> void:
	if motor_audio_controller == null:
		return

	var wheel_angular_velocity := INF
	for wheel in wheels:
		if wheel.is_driven:
			wheel_angular_velocity = minf(
				wheel_angular_velocity,
				absf(wheel.get_angular_velocity())
			)

	motor_audio_controller.rpm = motor.rpm


func _update_wheel_audio() -> void:
	var slip := 0.0
	for wheel in wheels:
		slip += wheel.get_slip_velocity().length()
	slip /= wheels.size()
	wheel_audio_controller.slip = slip


func _angular_velocity_to_rpm(velocity : float) -> float:
	return 60.0 * velocity / TAU
