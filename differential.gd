class_name Differential
extends Resource


@export_range(1.0, 10.0, 0.001, "or_greater") var limited_slip_ratio := 1.4


@export_subgroup("Input")

@export var is_locked := false

@export_range(0.0, 1.0) var torque_split := 0.5

@export var velocity_input := 0.0

@export var torque_input := 0.0

@export var velocity_feedback_1 := 0.0

@export var velocity_feedback_2 := 0.0

@export var torque_feedback_1 := 0.0

@export var torque_feedback_2 := 0.0


var _torque_output_1 := 0.0

var _torque_output_2 := 0.0

var _velocity_output_1 := 0.0

var _velocity_output_2 := 0.0


func get_torque_output_1() -> float:
	return _torque_output_1


func get_torque_output_2() -> float:
	return _torque_output_2


func get_velocity_output_1() -> float:
	return _velocity_output_1


func get_velocity_output_2() -> float:
	return _velocity_output_2


func update() -> void:
	var torque_fraction_1 := 1.0 - torque_split
	var torque_fraction_2 := 1.0 - torque_fraction_1

	if not is_locked and absf(velocity_feedback_1 - velocity_feedback_2) > 0.01:
		# limited slip implementation inspired by Jolt Physics
		# https://github.com/jrouwe/JoltPhysics/blob/515933138c6b16d661452fb907a8a9bbb71cb848/Jolt/Physics/Vehicle/VehicleDifferential.cpp#L42
		var unsigned_velocity_1 := absf(velocity_feedback_1)
		var unsigned_velocity_2 := absf(velocity_feedback_2)

		var min_velocity := minf(unsigned_velocity_1, unsigned_velocity_2)
		var max_velocity := maxf(unsigned_velocity_1, unsigned_velocity_2)

		var bias := minf((max_velocity / min_velocity - 1.0) / (limited_slip_ratio - 1.0), 1.0)
		torque_fraction_1 *= (1.0 - bias)
		torque_fraction_2 *= (1.0 - bias)
		if unsigned_velocity_1 < unsigned_velocity_2:
			torque_fraction_1 += bias
		else:
			torque_fraction_2 += bias

	_torque_output_1 = torque_fraction_1 * torque_input
	_torque_output_2 = torque_fraction_2 * torque_input
