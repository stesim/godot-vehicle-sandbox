class_name RayContactProbe
extends RayCast3D


@export var radius := 1.0

@export var width := 1.0

@export var distance := 1.0


var _contact_distance := 0.0


func _init() -> void:
	target_position = Vector3.ZERO
	exclude_parent = false
	enabled = false


func _ready() -> void:
	var parent_body := get_parent()
	while parent_body != null and not parent_body is RigidBody3D:
		parent_body = parent_body.get_parent()
	if parent_body != null:
		add_exception(parent_body)


func is_in_contact() -> bool:
	return is_colliding()


func get_contact_distance() -> float:
	return _contact_distance


func get_contact_point() -> Vector3:
	return get_collision_point()


func get_contact_normal() -> Vector3:
	return get_collision_normal()


func update() -> void:
	target_position = (distance + radius) * Vector3.DOWN
	force_raycast_update()
	_contact_distance = (global_position - get_collision_point()).dot(global_transform.basis.y) - radius if is_colliding() else 0.0
