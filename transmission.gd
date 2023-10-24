class_name Transmission
extends Resource


@export var gear_ratios : Array[float] = [-3.42, 0, 4.38, 2.86, 1.92, 1.36, 1, 0.82, 0.73]

@export var neutral_gear := 1

@export var axle_ratio := 3.75

@export var shift_time := 0.25

@export_subgroup("Input")

@export var gear := 0

@export var torque_input := 0.0


var _torque_output := 0.0

var _remaining_shift_time := 0.0


func update(delta : float) -> void:
	_remaining_shift_time = move_toward(_remaining_shift_time, 0.0, delta)

	_torque_output = get_current_gear_ratio() * torque_input


func get_current_gear_ratio() -> float:
	return gear_ratios[gear] * axle_ratio if not is_shifting() else 0.0


func get_torque_output() -> float:
	return _torque_output


func is_shifting() -> bool:
	return _remaining_shift_time > 0.0


func shift(new_gear : int) -> void:
	gear = clampi(new_gear, 0, gear_ratios.size() - 1)
	_remaining_shift_time = shift_time


func shift_relative(gear_difference : int) -> void:
	shift(gear + gear_difference)
