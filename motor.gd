class_name Motor
extends Resource


@export var rpm := 0.0 :
	set(value):
		rpm = clampf(value, 0.0, rpm_limit)

@export var throttle := 0.0

@export var peak_torque := 600.0

@export var rpm_limit := 5000.0

@export var normalization_rpm := 5000.0

@export var normalized_torque_curve : Curve


var _torque_output := 0.0


func update() -> void:
	if rpm >= rpm_limit:
		_torque_output = 0.0
	else:
		var normalized_rpm := rpm / normalization_rpm
		var nominal_torque := peak_torque * normalized_torque_curve.sample_baked(normalized_rpm)
		_torque_output = throttle * nominal_torque


func get_torque_output() -> float:
	return _torque_output


func get_power_output() -> float:
	var angular_velocity := rpm * TAU / 60.0
	return angular_velocity * get_torque_output()
