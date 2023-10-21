class_name Wheel
extends RayCast3D


@export var visuals : Node3D

@export var radius := 0.2

@export var rest_length := 0.3

@export var stiffness := 20000.0

@export var damping := 1500.0


func _ready() -> void:
	var total_length := rest_length + radius
	target_position.y = -total_length


func _physics_process(_delta : float) -> void:
	_update_visuals()


func _update_visuals() -> void:
	if is_colliding():
		visuals.global_position = get_collision_point()
	else:
		visuals.position = target_position

	visuals.position.y += radius
