class_name Transmission
extends Resource


@export var forward_gears : Array[float] = [4.38, 2.86, 1.92, 1.36, 1, 0.82, 0.73]

@export var reverse_gears : Array[float] = [-3.42]

@export_range(0.0, 6.0, 0.01, "or_greater") var axle_ratio := 3.75

@export_range(0.0, 1.0, 0.01, "or_greater") var shift_time := 0.25


@export_subgroup("Automation")

@export var is_automated := true

@export var torque_curve : Curve

@export_range(0.0, 4.0, 0.01, "or_greater")  var shift_interval := 1.0

@export_range(0.0, 1.0) var automated_upshift_rpm_threshold := 0.95

@export_range(0.0, 1.0) var automated_downshift_rpm_threshold := 0.9


@export_subgroup("Input")

@export var gear := 0

@export_range(0.0, 2000.0, 0.1, "or_greater") var torque_input := 0.0

@export_range(0.0, 1.0) var normalized_rpm_input := 0.0


var _torque_output := 0.0

var _remaining_shift_time := 0.0

var _remaining_shift_interval := 0.0


func update(delta : float) -> void:
	_remaining_shift_time = move_toward(_remaining_shift_time, 0.0, delta)

	if is_automated:
		_update_automation(delta)

	_torque_output = get_current_gear_ratio() * torque_input


func get_current_gear_ratio() -> float:
	return 0.0 if is_shifting() else axle_ratio * _get_gear_ratio(gear)


func get_torque_output() -> float:
	return _torque_output


func is_shifting() -> bool:
	return _remaining_shift_time > 0.0


func shift(new_gear : int) -> void:
	gear = clampi(new_gear, -reverse_gears.size(), forward_gears.size())
	_remaining_shift_time = shift_time
	_remaining_shift_interval = shift_interval


func shift_relative(gear_difference : int) -> void:
	shift(gear + gear_difference)


func _update_automation(delta : float) -> void:
	_remaining_shift_interval = move_toward(_remaining_shift_interval, 0.0, delta)

	if gear > 0:
		var optimal_gear := _find_gear_with_most_torque()
		if gear != optimal_gear and (is_zero_approx(_remaining_shift_interval) or normalized_rpm_input > 1.0):
			shift(optimal_gear)
		elif torque_input > 0.0 and normalized_rpm_input > automated_upshift_rpm_threshold and gear < forward_gears.size():
			shift_relative(+1)


func _find_gear_with_most_torque() -> int:
	var relative_rpm := normalized_rpm_input / _get_gear_ratio(gear)
	var max_torque := 0.0
	for i in forward_gears.size():
		var gear_ratio := _get_gear_ratio(i + 1)
		var normalized_rpm := relative_rpm * gear_ratio
		if normalized_rpm > automated_downshift_rpm_threshold:
			continue # shifting would result in rpms near or above engine's max rpm

		var gear_torque := gear_ratio * torque_curve.sample_baked(normalized_rpm)
		if max_torque < gear_torque:
			max_torque = gear_torque
		else:
			return i

	# this should never be reached
	return 0


func _get_gear_ratio(gear_index : int) -> float:
	var gear_ratio := (
		forward_gears[gear_index - 1] if gear_index > 0
		else reverse_gears[-gear_index - 1] if gear_index < 0
		else 0.0
	)
	return gear_ratio
