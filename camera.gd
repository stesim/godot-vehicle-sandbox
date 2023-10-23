extends Camera3D


@export var target : Node3D


func _process(_delta : float) -> void:
	if target != null:
		look_at(target.global_position)
