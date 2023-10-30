class_name Suspension
extends Resource


@export_range(0.0, 2.0, 0.01, "or_greater") var rest_length := 0.3

@export_range(0.0, 100000.0, 0.1, "or_greater") var stiffness := 40000.0

@export var stiffness_curve : Curve

@export_range(0.0, 10000.0, 0.1, "or_greater") var damping := 4000.0


func calculate_force(vehicle_state : PhysicsDirectBodyState3D, length : float, velocity : float) -> float:
	var spring_displacement := length - rest_length

	var effective_stiffness := stiffness
	if stiffness_curve != null:
		effective_stiffness *= stiffness_curve.sample_baked(-spring_displacement / rest_length)

	var max_displacement_velocity := rest_length / vehicle_state.step
	# HACK: avoid excessive forces due to large displacement velocities
	if absf(velocity) > max_displacement_velocity:
		velocity = signf(velocity) * max_displacement_velocity
	var force := -effective_stiffness * spring_displacement - damping * velocity
	# HACK: avoid negative force
	return maxf(0.0, force)
