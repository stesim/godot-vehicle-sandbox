extends Node


@export var wheel : Wheel

@export var smoke : GPUParticles3D

@export var slip_threshold := 8.0


func _physics_process(_delta : float) -> void:
	var slip := wheel.get_slip_velocity().length()
	if slip < slip_threshold:
		smoke.emitting = false
	else:
		smoke.emitting = true
		smoke.global_position = wheel.get_collision_point()
