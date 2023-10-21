class_name Vehicle
extends RigidBody3D


@export var wheels : Array[Wheel] = []


func _integrate_forces(state : PhysicsDirectBodyState3D) -> void:
	for wheel in wheels:
		_apply_suspension_force(state, wheel)


func _apply_suspension_force(state : PhysicsDirectBodyState3D, wheel : Wheel) -> void:
	if not wheel.is_colliding():
		return

	var up := wheel.global_transform.basis.y
	var contact_point := wheel.get_collision_point()
	var contact_distance := (wheel.global_position - contact_point).dot(up)
	var spring_displacement := contact_distance - (wheel.rest_length + wheel.radius)
	# NOTE: assumes the ground/collider is stationary
	var displacement_velocity := state.get_velocity_at_local_position(contact_point).dot(up)

	var force_magnitude := -wheel.stiffness * spring_displacement - wheel.damping * displacement_velocity

	var force := force_magnitude * up
	var force_position := contact_point - global_position
	state.apply_force(force, force_position)
