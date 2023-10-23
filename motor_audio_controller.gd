class_name MotorAudioController
extends Node


@export var rpm := 0.0

@export var baseline_rpm := 800.0

@export var pitch_scale := 1600.0

@export var drive_loop : AudioStreamPlayer3D


func _ready() -> void:
	var original_volume := drive_loop.volume_db
	drive_loop.volume_db = linear_to_db(0.0)
	drive_loop.play()
	await get_tree().process_frame
	drive_loop.volume_db = original_volume


func _physics_process(_delta : float) -> void:
	if rpm < 10.0:
		drive_loop.stream_paused = true
	else:
		drive_loop.pitch_scale = maxf(0.001, 1.0 + (rpm - baseline_rpm) / pitch_scale)
		drive_loop.stream_paused = false
