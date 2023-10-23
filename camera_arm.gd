extends Node3D


@export var stiffness := 8.0


@onready var _target : Node3D = get_parent()

@onready var _offset := global_position - _target.global_position


func _ready() -> void:
	top_level = true
	_physics_process(1.0 / stiffness)


func _physics_process(delta : float) -> void:
	var ideal_position := _target.global_position + _offset.rotated(Vector3.UP, _target.global_rotation.y)
	var interpolated_position := global_position.lerp(ideal_position, delta * stiffness)
	look_at_from_position(interpolated_position, _target.global_position)
