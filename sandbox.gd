extends Node3D


const AccelerationTimer := preload("acceleration_timer.gd")


@export var camera_crane : CameraCrane

@export var camera_view := 0

@export var camera_views : Array[Node3D] = []


@onready var _vehicle : Vehicle = $vehicle

@onready var _speed_label : Label = %speed_label

@onready var _gear_label : Label = %gear_label

@onready var _rpm_label : Label = %rpm_label

@onready var _slip_info := $hud/slip_info

@onready var _acceleration_timer : AccelerationTimer = $acceleration_timer

@onready var _acceleration_timer_interface := $hud/acceleration_timer


func _ready() -> void:
	camera_crane.snap_to_position(camera_views[camera_view])

	_initialize_timer_interface()


func _unhandled_input(event : InputEvent) -> void:
	if event.is_action_pressed(&"change_camera"):
		_cycle_camera_view()
	if event.is_action_pressed(&"toggle_timer"):
		_acceleration_timer.toggle()


func _process(_delta : float) -> void:
	_speed_label.text = str(absi(int(_vehicle.get_speed(Vehicle.KILOMETERS_PER_HOUR))))
	var gear := _vehicle.transmission.gear - _vehicle.transmission.neutral_gear
	_gear_label.text = "--" if _vehicle.transmission.is_shifting() else "N" if gear == 0 else "R" if gear < 0 else "D" + str(gear)
	_rpm_label.text = str(int(_vehicle.motor.rpm))
	for i in _vehicle.wheels.size():
		_slip_info.get_child(i).text = "%+.2f" % _vehicle.wheels[i].get_slip_velocity().x


func _cycle_camera_view() -> void:
	var current_view_index := camera_views.find(camera_crane.get_current_camera_position())
	var next_view_index := (current_view_index + 1) % camera_views.size()
	camera_crane.transition_to_position(camera_views[next_view_index])


func _initialize_timer_interface() -> void:
	_acceleration_timer_interface.hide()

	for target_speed in _acceleration_timer.target_speeds:
		var speed_label := Label.new()
		speed_label.text = "0 - %d" % int(target_speed)
		var time_label := Label.new()
		_acceleration_timer_interface.add_child(speed_label)
		_acceleration_timer_interface.add_child(time_label)

	_acceleration_timer.activated.connect(func():
		for i in _acceleration_timer.target_speeds.size():
			var time_label : Label = _acceleration_timer_interface.get_child(3 + 2 * i)
			time_label.text = "--"
		_acceleration_timer_interface.show()
	)

	_acceleration_timer.time_recorded.connect(func(index : int, value : float):
		var time_label : Label = _acceleration_timer_interface.get_child(3 + 2 * index)
		time_label.text = "%.1f" % value
	)
