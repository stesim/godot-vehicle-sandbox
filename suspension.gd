class_name Suspension
extends Resource


@export var rest_length := 0.3

@export var stiffness := 40000.0

@export var stiffness_curve : Curve

@export var damping := 4000.0


func calculate_force(_vehicle_state : PhysicsDirectBodyState3D, wheel : Wheel) -> float:
	if not wheel.is_colliding():
		return 0.0

	var up := wheel.global_transform.basis.y
	var contact_point := wheel.get_collision_point()
	var contact_distance := (wheel.global_position - contact_point).dot(up)
	var spring_displacement := contact_distance - (rest_length + wheel.radius)

	var effective_stiffness := stiffness
	if stiffness_curve != null:
		effective_stiffness *= stiffness_curve.sample_baked(-spring_displacement / rest_length)

	var displacement_velocity := wheel.get_contact_velocity().dot(up)
	var force := -effective_stiffness * spring_displacement - damping * displacement_velocity
	return force
