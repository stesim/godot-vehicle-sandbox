extends Node3D


@export var camera_crane : CameraCrane

@export var camera_view := 0

@export var camera_views : Array[Node3D] = []


@onready var _vehicle : Vehicle = $vehicle

@onready var _speed_label : Label = %speed_label

@onready var _gear_label : Label = %gear_label

@onready var _rpm_label : Label = %rpm_label

@onready var _slip_info := %slip_info


func _ready() -> void:
	camera_crane.snap_to_position(camera_views[camera_view])


func _unhandled_input(event : InputEvent) -> void:
	if event.is_action_pressed(&"change_camera"):
		_cycle_camera_view()


func _process(_delta : float) -> void:
	var speed := absf(_vehicle.linear_velocity.dot(_vehicle.global_transform.basis.z))
	_speed_label.text = str(int(speed * 3.6))
	var gear := _vehicle.transmission.gear - _vehicle.transmission.neutral_gear
	_gear_label.text = "--" if _vehicle.transmission.is_shifting() else "N" if gear == 0 else "R" if gear < 0 else "D" + str(gear)
	_rpm_label.text = str(int(_vehicle.motor.rpm))
	for i in _vehicle.wheels.size():
		_slip_info.get_child(i).text = "%+.2f" % _vehicle.wheels[i].get_slip_velocity().x


func _cycle_camera_view() -> void:
	var current_view_index := camera_views.find(camera_crane.get_current_camera_position())
	var next_view_index := (current_view_index + 1) % camera_views.size()
	camera_crane.transition_to_position(camera_views[next_view_index])
