extends Node3D


@export var camera_crane : CameraCrane

@export var camera_views : Array[Node3D] = []


func _ready() -> void:
	camera_crane.snap_to_position(camera_views[0])


func _unhandled_input(event : InputEvent) -> void:
	if event.is_action_pressed(&"change_camera"):
		_cycle_camera_view()


func _cycle_camera_view() -> void:
	var current_view_index := camera_views.find(camera_crane.get_current_camera_position())
	var next_view_index := (current_view_index + 1) % camera_views.size()
	camera_crane.transition_to_position(camera_views[next_view_index])
