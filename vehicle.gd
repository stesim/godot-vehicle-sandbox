class_name Vehicle
extends RigidBody3D


@export var max_steering_angle := deg_to_rad(30.0)

@export var max_brake_torque := 4000.0

@export var max_handbrake_torque := 1550.0

@export var auto_brake_threshold := 0.25

@export var frontal_area := 2.3

@export var drag_coefficient := 0.3

@export var air_density := 1.204

@export var motor : Motor

@export var transmission : Transmission

@export var front_differential : Differential

@export var center_differential : Differential

@export var rear_differential : Differential

@export var wheels : Array[Wheel] = []

@export var input_speed := 6.0

@export var motor_audio_controller : MotorAudioController

@export var wheel_audio_controller : TireAudioController


var _engine_input := 0.0

var _steering_input := 0.0

var _brake_input := 0.0

var _handbrake_input := 0.0


func _ready() -> void:
	pass


func _process(delta : float) -> void:
	_update_inputs(delta)
	_update_audio()


func _integrate_forces(state : PhysicsDirectBodyState3D) -> void:
	_apply_drive_input(state.step)
	_apply_suspension_forces(state)
	_apply_tire_forces(state)
	_apply_drag(state)


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
			var virtual_mass := mass / wheels.size()
			wheel.apply_bottom_out_force(state, virtual_mass)


func _apply_tire_forces(state : PhysicsDirectBodyState3D) -> void:
	for wheel in wheels:
		wheel.apply_drive_forces(state)
		wheel.apply_rolling_resistance(state)


func _update_inputs(delta : float) -> void:
	_engine_input = move_toward(_engine_input, Input.get_action_strength(&"accelerate"), delta * input_speed)
	_brake_input = move_toward(_brake_input, Input.get_action_strength(&"brake"), delta * input_speed)
	_handbrake_input = move_toward(_handbrake_input, Input.get_action_strength(&"handbrake"), delta * input_speed)
	_steering_input = move_toward(_steering_input, Input.get_axis(&"steer_right", &"steer_left"), delta * input_speed)

	_apply_steering_input()


func _apply_drive_input(delta : float) -> void:
	var brake_torque := 0.0
	var handbrake_torque := _handbrake_input * max_handbrake_torque

	motor.throttle = _engine_input
	brake_torque = _brake_input * max_brake_torque

	var gear_ratio := transmission.get_current_gear_ratio()
	motor.rpm_feedback = 0.0
	motor.is_engaged = not is_zero_approx(gear_ratio)
	if motor.is_engaged:
		var feedback_rpm := INF
		for wheel in wheels:
			if wheel.is_driven:
				feedback_rpm = minf(feedback_rpm, gear_ratio * wheel.get_rpm())
		motor.rpm_feedback = feedback_rpm

	motor.update(delta)
	transmission.torque_input = motor.get_torque_output()
	transmission.update(delta)

	if motor.is_engaged:
		# HACK: assumes wheel positions and 4-wheel drive
		var feedback_fl := wheels[0].get_torque_feedback()
		var feedback_fr := wheels[1].get_torque_feedback()
		var feedback_rl := wheels[2].get_torque_feedback()
		var feedback_rr := wheels[3].get_torque_feedback()

		var front_feedback := feedback_fl + feedback_fr
		var rear_feedback := feedback_rl + feedback_rr

		center_differential.torque_input = transmission.get_torque_output()
		center_differential.velocity_feedback_1 = 0.5 * (wheels[2].get_angular_velocity() + wheels[3].get_angular_velocity())
		center_differential.velocity_feedback_2 = 0.5 * (wheels[0].get_angular_velocity() + wheels[1].get_angular_velocity())
		center_differential.torque_feedback_1 = rear_feedback
		center_differential.torque_feedback_2 = front_feedback
		center_differential.update()

		rear_differential.torque_input = center_differential.get_torque_output_1()
		rear_differential.velocity_feedback_1 = wheels[2].get_angular_velocity()
		rear_differential.velocity_feedback_2 = wheels[3].get_angular_velocity()
		rear_differential.torque_feedback_1 = feedback_rl
		rear_differential.torque_feedback_2 = feedback_rr
		rear_differential.update()

		front_differential.torque_input = center_differential.get_torque_output_2()
		front_differential.velocity_feedback_1 = wheels[0].get_angular_velocity()
		front_differential.velocity_feedback_2 = wheels[1].get_angular_velocity()
		front_differential.torque_feedback_1 = feedback_fl
		front_differential.torque_feedback_2 = feedback_fr
		front_differential.update()

		wheels[0].drive_torque = front_differential.get_torque_output_1()
		wheels[1].drive_torque = front_differential.get_torque_output_2()
		wheels[2].drive_torque = rear_differential.get_torque_output_1()
		wheels[3].drive_torque = rear_differential.get_torque_output_2()
	else:
		wheels[0].drive_torque = 0.0
		wheels[1].drive_torque = 0.0
		wheels[2].drive_torque = 0.0
		wheels[3].drive_torque = 0.0

	for wheel in wheels:
		wheel.brake_torque = maxf(brake_torque, handbrake_torque) if wheel.has_handbrake else brake_torque
		if wheel.is_driven:
			wheel.drivetrain_inertia = absf(gear_ratio) * motor.inertia


func _apply_steering_input() -> void:
	var steering_angle := _steering_input * max_steering_angle
	if is_zero_approx(steering_angle):
		for wheel in wheels:
			if wheel.is_steering:
				wheel.steering_angle = steering_angle
		return

	var turn_center := _calculate_turn_center()
	for wheel in wheels:
		if wheel.is_steering:
			var turn_center_offset := wheel.position - turn_center
			var turn_radius := -turn_center_offset.z / tan(steering_angle)
			var ackermann_angle = atan(-turn_center_offset.z / (turn_radius + turn_center_offset.x))
			wheel.steering_angle = ackermann_angle


func _apply_drag(state : PhysicsDirectBodyState3D) -> void:
	var forward := -state.transform.basis.z
	var longitudinal_velocity := state.linear_velocity.dot(forward)
	var drag_force := 0.5 * air_density * longitudinal_velocity * longitudinal_velocity * drag_coefficient * frontal_area
	state.apply_central_force(-drag_force * forward)


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
	if motor_audio_controller != null:
		motor_audio_controller.rpm = motor.rpm


func _update_wheel_audio() -> void:
	var slip := 0.0
	for wheel in wheels:
		slip += wheel.get_slip_velocity().length()
	slip /= wheels.size()
	wheel_audio_controller.slip = slip


func _angular_velocity_to_rpm(velocity : float) -> float:
	return 60.0 * velocity / TAU
