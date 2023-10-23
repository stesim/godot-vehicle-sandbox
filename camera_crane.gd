class_name CameraCrane
extends Node3D


@export var subject : Node3D

@export var speed := 4.0

@export var speed_boost := 64.0


var _target : Node3D

var _boost_time := 0.0


func get_current_camera_position() -> Node3D:
	return _target if _target != null else (get_parent() as Node3D)


func snap_to_position(target : Node3D) -> void:
	if target == get_current_camera_position():
		return
	_target = target
	_attach_to_target()


func transition_to_position(target : Node3D) -> void:
	if target == get_current_camera_position():
		return
	_target = target
	_boost_time = 0.0
	_detach_from_parent()


func _physics_process(delta: float) -> void:
	if _target == null:
		return

	_boost_time += delta
	_step_towards_target(delta)
	var distance := global_position.distance_to(_target.global_position)
	if distance < 0.01:
		_attach_to_target()


func _step_towards_target(delta : float) -> void:
	var weight := delta * (speed + speed_boost * _boost_time * _boost_time)
	if subject != null:
		var target_offset := _target.global_position - subject.global_position
		var own_offset := global_position - subject.global_position
		global_position = subject.global_position + own_offset.slerp(target_offset, weight)
	else:
		global_position = global_position.lerp(_target.global_position, weight)
	global_transform.basis = global_transform.basis.slerp(_target.global_transform.basis, weight).orthonormalized()


func _attach_to_target() -> void:
	reparent(_target, false)
	_target = null
	_boost_time = 0.0
	set_identity()


func _detach_from_parent() -> void:
	var world_node := get_tree().current_scene
	if get_parent() != world_node:
		reparent(world_node, true)
