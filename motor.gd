class_name Motor
extends Resource


@export var idle_rpm := 800.0

@export var rpm_limit := 5000.0

@export var peak_torque := 600.0

@export var normalization_rpm := 5000.0

@export var normalized_torque_curve : Curve

@export var throttle_response_torque := 800.0

@export var friction_torque := 150.0

@export var inertia := 1.0

@export_subgroup("Input")

@export var throttle := 0.0

@export var rpm := 0.0 :
	set(value):
		rpm = clampf(value, idle_rpm, rpm_limit)

@export var rpm_feedback := -1.0


var _torque_output := 0.0


func update(delta : float) -> void:
	if rpm_feedback >= 0.0:
		rpm = rpm_feedback
	else:
		var total_friction_torque := -friction_torque + throttle * throttle_response_torque
		var rpm_delta := total_friction_torque / inertia * delta / TAU * 60.0
		rpm = clampf(rpm + rpm_delta, idle_rpm, rpm_limit)

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
