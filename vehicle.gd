class_name Vehicle
extends RigidBody3D


@export var wheels : Array[Wheel] = []


func _integrate_forces(state : PhysicsDirectBodyState3D) -> void:
	_apply_suspension_forces(state)


func _apply_suspension_forces(state : PhysicsDirectBodyState3D) -> void:
	for wheel in wheels:
		wheel.apply_suspension_force(state)

	for wheel in wheels:
		if wheel.is_bottoming_out():
			# NOTE: assumes uniform mass distribution, but appears to work reasonably well
			# TODO: more accurate solution?
			var virtual_mass = mass / wheels.size()
			wheel.apply_bottom_out_impulse(state, virtual_mass)
