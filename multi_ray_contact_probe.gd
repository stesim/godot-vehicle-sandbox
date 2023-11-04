class_name MultiRayContactProbe
extends RayCast3D


@export var radius := 1.0

@export var width := 1.0

@export var distance := 1.0

@export var ray_count := 7 :
	set(value):
		ray_count = value
		_normalized_offsets.resize(ray_count)
		for i in ray_count:
			_normalized_offsets[i] = sin(PI * (float(i) / (ray_count - 1) - 0.5))


var _normalized_offsets := PackedFloat32Array()

var _is_in_contact := false

var _contact_distance := 0.0

var _contact_point := Vector3.ZERO

var _contact_normal := Vector3.ZERO


func _init() -> void:
	target_position = Vector3.ZERO
	exclude_parent = false
	enabled = false
	ray_count = ray_count


func _ready() -> void:
	var parent_body := get_parent()
	while parent_body != null and not parent_body is RigidBody3D:
		parent_body = parent_body.get_parent()
	if parent_body != null:
		add_exception(parent_body)


func is_in_contact() -> bool:
	return _is_in_contact


func get_contact_distance() -> float:
	return _contact_distance


func get_contact_point() -> Vector3:
	return _contact_point


func get_contact_normal() -> Vector3:
	return _contact_normal


func update() -> void:
	_is_in_contact = false
	_contact_point = Vector3.ZERO
	_contact_normal = Vector3.ZERO
	_contact_distance = INF

	for normalized_offset in _normalized_offsets:
		var offset := radius * normalized_offset
		var radius_part := sqrt(absf(radius * radius - offset * offset))
		target_position = (distance + radius_part) * Vector3.DOWN
		position.z = offset
		force_raycast_update()
		if not is_colliding():
			continue
		var collision_distance := (global_position - get_collision_point()).dot(global_transform.basis.y) - radius_part
		if collision_distance < _contact_distance:
			_is_in_contact = true
			_contact_point = get_collision_point()
			_contact_normal = get_collision_normal()
			_contact_distance = collision_distance

	if not _is_in_contact:
		_contact_distance = 0.0
