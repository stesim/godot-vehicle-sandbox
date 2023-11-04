class_name RigidBodyContactProbe
extends Node3D


@export var radius := 1.0 :
	set(value):
		radius = value
		_shape.shape.radius = radius
		_update_body_mass()

@export var width := 1.0 :
	set(value):
		width = value
		_shape.shape.height = width
		_update_body_mass()

@export var distance := 1.0 :
	set(value):
		distance = value
		_joint.limit_lower = -distance

@export var density := 250.0 :
	set(value):
		density = value
		_update_body_mass()


static var _physics_material := _create_physics_material()


var _shape := _create_shape()

var _body := _create_body()

var _joint := _create_joint()

var _contact_distance := 0.0

var _contact_point := Vector3.ZERO

var _contact_normal := Vector3.ZERO


func is_in_contact() -> bool:
	return _body.get_contact_count() > 0


func get_contact_distance() -> float:
	return _contact_distance


func get_contact_point() -> Vector3:
	return _contact_point


func get_contact_normal() -> Vector3:
	return _contact_normal


func update() -> void:
	if _body.get_contact_count() == 0:
		_contact_distance = 0.0
		_contact_point = Vector3.ZERO
		_contact_normal = Vector3.ZERO
		return

	_contact_distance = (global_position - _body.global_position).dot(global_transform.basis.y)
	_contact_point = global_position - _contact_distance * global_transform.basis.y
	# FIXME: determine actual normal
	_contact_normal = global_transform.basis.y


func _ready() -> void:
	add_child(_body)
	add_child(_joint)

	var parent_body := get_parent()
	while parent_body != null and not parent_body is RigidBody3D:
		parent_body = parent_body.get_parent()
	_joint.node_a = parent_body.get_path()
	_joint.node_b = _body.get_path()


func _update_body_mass() -> void:
	_body.mass = PI * radius * radius * width * density


func _create_shape() -> CollisionShape3D:
	var shape := CollisionShape3D.new()
	shape.shape = CylinderShape3D.new()
	shape.rotation.z = 0.5 * PI
	return shape


func _create_body() -> RigidBody3D:
	var body := RigidBody3D.new()
	body.top_level = true
	body.physics_material_override = _physics_material
	body.contact_monitor = true
	body.max_contacts_reported = 1
	body.linear_damp_mode = RigidBody3D.DAMP_MODE_REPLACE
	body.linear_damp = 0.0
	body.angular_damp_mode = RigidBody3D.DAMP_MODE_REPLACE
	body.angular_damp = 0.0
	#body.continuous_cd = true
	#body.collision_priority = 100
	body.add_child(_shape)
	return body


func _create_joint() -> JoltSliderJoint3D:
	var joint := JoltSliderJoint3D.new()
	joint.rotation.z = 0.5 * PI
	joint.limit_enabled = true
	joint.top_level = true
	return joint


static func _create_physics_material() -> PhysicsMaterial:
	var material := PhysicsMaterial.new()
	material.friction = 0.0
	return material
