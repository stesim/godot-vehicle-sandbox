extends Node


signal activated()

signal time_recorded(index : int, value : float)

signal stopped()


@export var active := false :
	set(value):
		if active != value:
			active = value
			if active:
				_reset()
				activated.emit()
			else:
				stopped.emit()
			set_physics_process(active and not target_speeds.is_empty())

@export var vehicle : Vehicle

@export var target_speeds : Array[float] = [80.0, 100.0, 160.0]


var _elapsed_time := 0.0

var _target_index := 0

var _results := PackedFloat32Array()


func start() -> void:
	active = false
	active = true


func stop() -> void:
	active = false


func toggle() -> void:
	active = not active


func get_results() -> PackedFloat32Array:
	return _results


func _ready() -> void:
	set_physics_process(active and not target_speeds.is_empty())


func _physics_process(delta : float) -> void:
	if target_speeds.is_empty():
		return
	var speed := vehicle.get_speed(Vehicle.KILOMETERS_PER_HOUR)
	if speed < 0.01:
		return

	_elapsed_time += delta

	if speed >= target_speeds[_target_index]:
		_results[_target_index] = _elapsed_time
		time_recorded.emit(_target_index, _elapsed_time)
		_target_index += 1
		if _target_index >= target_speeds.size():
			stop()


func _reset() -> void:
	_elapsed_time = 0.0
	_target_index = 0
	_results.resize(target_speeds.size())
	_results.fill(-1.0)
