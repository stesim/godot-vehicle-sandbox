class_name DebugVectorOverlay
extends Node3D


@export var vector := Vector3.ZERO
@export var color := Color.GREEN
@export var width := 2.0


@onready var _camera : Camera3D = get_viewport().get_camera_3d()


var _arrow_head_points := PackedVector2Array([
	Vector2(0.0, 0.0),
	Vector2(-12.0, -6.0),
	Vector2(-12.0, 6.0),
])


func _init() -> void:
	add_to_group(&"debug_overlays")


func draw(canvas : CanvasItem) -> void:
	var start := _camera.unproject_position(global_position)
	var end := _camera.unproject_position(global_position + vector)
	canvas.draw_line(start, end, color, width)

	if start.distance_squared_to(end) >= 36.0:
		canvas.draw_set_transform(end, start.angle_to_point(end))
		canvas.draw_colored_polygon(_arrow_head_points, color)
		canvas.draw_set_transform(Vector2.ZERO)
