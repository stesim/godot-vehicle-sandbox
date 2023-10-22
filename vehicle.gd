class_name Vehicle
extends RigidBody3D


@export var max_engine_torque := 4.0 * 3.0 * 500.0

@export var wheels : Array[Wheel] = []

@export var input_speed := 2.0


var _engine_input := 0.0


func _process(delta : float) -> void:
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
			var virtual_mass = mass / wheels.size()
			wheel.apply_bottom_out_impulse(state, virtual_mass)


func _apply_tire_forces(state : PhysicsDirectBodyState3D) -> void:
	for wheel in wheels:
		wheel.apply_tire_forces(state)
