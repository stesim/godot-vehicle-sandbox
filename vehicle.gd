class_name Vehicle
extends RigidBody3D


const METERS_PER_SECOND := 1.0

const KILOMETERS_PER_HOUR := 3.6


@export_range(0.0, 90.0, 0.01, "radians") var max_steering_angle := deg_to_rad(30.0)

@export_range(0.0, 10.0, 0.01, "or_greater") var frontal_area := 2.3

@export_range(0.0, 2.0, 0.01, "or_greater") var drag_coefficient := 0.3

@export_range(0.0, 2.0, 0.01, "or_greater") var air_density := 1.204

@export var motor : Motor

@export var transmission : Transmission

@export var center_differential : Differential

@export_range(0.0, 20.0, 0.01, "or_greater") var input_speed := 6.0

@export var motor_audio_controller : MotorAudioController

@export var wheel_audio_controller : TireAudioController


var _axles : Array[Axle] = []

var _engine_input := 0.0

var _steering_input := 0.0

var _brake_input := 0.0

var _handbrake_input := 0.0


func get_speed(unit := METERS_PER_SECOND) -> float:
	return -linear_velocity.dot(global_transform.basis.z) * unit


func _init() -> void:
	child_entered_tree.connect(_on_child_entered)
	child_exiting_tree.connect(_on_child_exiting)


func _ready() -> void:
	if transmission != null and motor != null:
		transmission.torque_curve = motor.power_torque_curve


func _process(delta : float) -> void:
	_update_inputs(delta)
	_update_audio()


func _integrate_forces(state : PhysicsDirectBodyState3D) -> void:
	if motor != null:
		_apply_drive_input(state.step)
	for axle in _axles:
		axle.update(state, mass / _axles.size())
	_apply_drag(state)


func _unhandled_input(event : InputEvent) -> void:
	if transmission != null:
		if event.is_action_pressed(&"shift_up"):
			transmission.shift_relative(+1)
		elif event.is_action_pressed(&"shift_down"):
			transmission.shift_relative(-1)


func _update_inputs(delta : float) -> void:
	_engine_input = move_toward(_engine_input, Input.get_action_strength(&"accelerate"), delta * input_speed)
	_brake_input = move_toward(_brake_input, Input.get_action_strength(&"brake"), delta * input_speed)
	_handbrake_input = move_toward(_handbrake_input, Input.get_action_strength(&"handbrake"), delta * input_speed)
	_steering_input = move_toward(_steering_input, Input.get_axis(&"steer_right", &"steer_left"), delta * input_speed)

	_apply_steering_input()


func _apply_drive_input(delta : float) -> void:
	motor.throttle = _engine_input
	if transmission != null and transmission.is_shifting():
		motor.throttle = 0.0

	var gear_ratio := transmission.get_current_gear_ratio() if transmission != null else 1.0
	motor.is_engaged = not is_zero_approx(gear_ratio)

	if not motor.is_engaged:
		motor.rpm_feedback = 0.0
	else:
		var feedback_rpm := INF
		for axle in _axles:
			if axle.is_driven:
				for wheel in axle.get_wheels():
					feedback_rpm = minf(feedback_rpm, gear_ratio * wheel.get_rpm())
		motor.rpm_feedback = feedback_rpm
		motor.update(delta)

	var torque_output = motor.get_torque_output()

	if transmission != null:
		transmission.torque_input = motor.get_torque_output()
		transmission.normalized_rpm_input = motor.rpm / motor.normalization_rpm
		transmission.update(delta)
		torque_output = transmission.get_torque_output()

	if center_differential != null and _axles.size() == 2 and motor.is_engaged:
		center_differential.torque_input = torque_output
		center_differential.velocity_feedback_1 = _axles[0].get_angular_velocity_feedback()
		center_differential.velocity_feedback_2 = _axles[1].get_angular_velocity_feedback()
		center_differential.torque_feedback_1 = _axles[0].get_torque_feedback()
		center_differential.torque_feedback_2 = _axles[1].get_torque_feedback()
		center_differential.update()

		_axles[0].torque_input = center_differential.get_torque_output_1()
		_axles[1].torque_input = center_differential.get_torque_output_2()
	else:
		for axle in _axles:
			axle.torque_input = torque_output / _axles.size()

	for axle in _axles:
		for wheel in axle.get_wheels():
			wheel.brake_input = _brake_input
			wheel.handbrake_input = _handbrake_input
			if axle.is_driven:
				wheel.drivetrain_inertia = absf(gear_ratio) * motor.inertia


func _apply_steering_input() -> void:
	var steering_angle := _steering_input * max_steering_angle
	var turn_center := _calculate_turn_center()
	for axle in _axles:
		if axle.is_steering:
			axle.steering_angle = steering_angle
			axle.turn_center = turn_center


func _apply_drag(state : PhysicsDirectBodyState3D) -> void:
	var forward := -state.transform.basis.z
	var longitudinal_velocity := state.linear_velocity.dot(forward)
	var drag_force := 0.5 * air_density * longitudinal_velocity * longitudinal_velocity * drag_coefficient * frontal_area
	state.apply_central_force(-drag_force * forward)


func _calculate_turn_center() -> Vector3:
	var turn_center := Vector3.ZERO
	var num_non_steering_wheels := 0
	for axle in _axles:
		if not axle.is_steering:
			for wheel in axle.get_wheels():
				turn_center += wheel.position
				num_non_steering_wheels += 1
	turn_center /= num_non_steering_wheels
	return turn_center


func _update_audio() -> void:
	if motor_audio_controller != null and motor != null:
		_update_motor_audio()
	if wheel_audio_controller != null:
		_update_wheel_audio()


func _update_motor_audio() -> void:
	motor_audio_controller.rpm = motor.rpm


func _update_wheel_audio() -> void:
	var num_wheels := 0
	var slip := 0.0
	for axle in _axles:
		for wheel in axle.get_wheels():
			slip += wheel.get_slip_velocity().length()
			num_wheels += 1
	slip /= num_wheels
	wheel_audio_controller.slip = slip


func _on_child_entered(child : Node) -> void:
	if child is Axle:
		_axles.push_back(child)


func _on_child_exiting(child : Node) -> void:
	if child is Axle:
		_axles.erase(child)
