class_name SmoothFollow
extends Node


@export var follower : Node3D
@export var target : Node3D
@export var speed := 5.0


@onready var _offset := follower.global_position - target.global_position


func _physics_process(delta : float) -> void:
	var ideal_position := target.global_position + _offset.rotated(Vector3.UP, target.global_rotation.y)
	var interpolated_position := follower.global_position.lerp(ideal_position, delta * speed)
	follower.look_at_from_position(interpolated_position, target.global_position)
