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

@export var target_speeds : Array[float] = [100.0]

@export var reference_times : Array[float] = [8.0]

@export var trigger_speed := 0.01


var _is_recording := false

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
	var speed := vehicle.get_speed(Vehicle.KILOMETERS_PER_HOUR)
	if not _is_recording and speed > trigger_speed:
		_is_recording = true
	var lower_threshold := target_speeds[_target_index - 1] if _target_index > 0 else trigger_speed
	if _is_recording and speed < lower_threshold:
		stop()

	if _is_recording:
		_elapsed_time += delta

		if speed >= target_speeds[_target_index]:
			_results[_target_index] = _elapsed_time
			time_recorded.emit(_target_index, _elapsed_time)
			_target_index += 1
			if _target_index >= target_speeds.size():
				stop()


func _reset() -> void:
	_is_recording = false
	_elapsed_time = 0.0
	_target_index = 0
	_results.resize(target_speeds.size())
	_results.fill(-1.0)
