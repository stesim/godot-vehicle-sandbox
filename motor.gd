class_name Motor
extends Resource


@export var idle_rpm := 800.0

@export var rpm_limit := 5000.0

@export var rpm_limiter_cutoff_time := 0.15

@export var peak_torque := 600.0

@export var normalization_rpm := 5000.0

@export var power_torque_curve : Curve

@export var brake_torque_curve : Curve

@export var throttle_response_torque := 800.0

@export var friction_torque := 150.0

@export var inertia := 1.0


@export_subgroup("Input")

@export_range(0.0, 1.0) var throttle := 0.0

@export var rpm := 0.0 :
	set(value):
		rpm = clampf(value, idle_rpm, rpm_limit)

@export var rpm_feedback := 0.0

@export var is_engaged := false


var _torque_output := 0.0

var _remaining_cutoff_time := 0.0


func update(delta : float) -> void:
	_torque_output = 0.0
	_remaining_cutoff_time = move_toward(_remaining_cutoff_time, 0.0, delta)

	if is_engaged:
		rpm = rpm_feedback
	else:
		var effective_throttle := 0.0 if _is_cut_off() else throttle
		var total_torque := effective_throttle * throttle_response_torque - friction_torque
		var rpm_delta := total_torque / inertia * delta / TAU * 60.0
		rpm = maxf(idle_rpm, rpm + rpm_delta)

	if rpm >= rpm_limit:
		_cut_off()

	if is_engaged:
		var effective_throttle := 0.0 if _is_cut_off() else throttle
		var normalized_rpm := rpm / normalization_rpm
		var power_torque := peak_torque * power_torque_curve.sample_baked(normalized_rpm)
		var brake_torque := -peak_torque * brake_torque_curve.sample_baked(normalized_rpm)
		var net_torque := lerpf(brake_torque, power_torque, effective_throttle)
		_torque_output = net_torque
		if rpm <= idle_rpm and _torque_output < 0.0:
			_torque_output = 0.0


func get_torque_output() -> float:
	return _torque_output


func get_angular_velocity() -> float:
	return rpm * TAU / 60.0


func _cut_off() -> void:
	_remaining_cutoff_time = rpm_limiter_cutoff_time


func _is_cut_off() -> bool:
	return not is_zero_approx(_remaining_cutoff_time)


func _to_angular_velocity(rpms : float) -> float:
	return rpms * TAU / 60.0


func _to_rpm(angular_velocity : float) -> float:
	return angular_velocity / TAU * 60.0
