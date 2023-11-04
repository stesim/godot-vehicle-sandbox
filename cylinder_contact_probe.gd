class_name CylinderContactProbe
extends ShapeCast3D


@export var radius := 1.0 :
	set(value):
		radius = value
		shape.radius = radius

@export var width := 1.0 :
	set(value):
		width = value
		shape.height = width

@export var distance := 1.0 :
	set(value):
		distance = value
		target_position.x = distance


var _contact_distance := 0.0

var _contact_point := Vector3.ZERO

var _contact_normal := Vector3.ZERO


func _init() -> void:
	shape = CylinderShape3D.new()
	rotation.z = -0.5 * PI
	target_position = Vector3.ZERO
	exclude_parent = false
	enabled = false
	max_results = 4


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
	return _contact_point


func get_contact_normal() -> Vector3:
	return _contact_normal


func update() -> void:
	force_shapecast_update()

	if not is_colliding():
		_contact_distance = 0.0
		_contact_point = Vector3.ZERO
		_contact_normal = Vector3.ZERO
		return

	_contact_distance = INF
	for i in get_collision_count():
		var local_collision_point := to_local(get_collision_point(i))
		var collision_distance := local_collision_point.x - sqrt(absf(radius * radius - local_collision_point.z * local_collision_point.z))
		if collision_distance < _contact_distance:
			_contact_point = get_collision_point(i)
			_contact_normal = get_collision_normal(i)
			_contact_distance = collision_distance

	# HACK: move contact point to center of contact patch to avoid lateral drift if the reported
	#       collision point is, for example, on the right side for all wheels
	var right := global_transform.basis.y
	_contact_point = global_position + (_contact_point - global_position).slide(right)
