class_name TireAudioController
extends Node


@export var slide_loop : AudioStreamPlayer3D

@export var slip := 0.0

@export var slip_min := 1.0

@export var slip_max := 16.0

@export var slip_pitch_min := 0.75

@export var slip_pitch_max := 1.125

@export var slip_pitch_curve : Curve

@export var slip_volume_min := 0.5

@export var slip_volume_max := 1.25

@export var slip_volume_curve : Curve

@export var pitch_transition_duration := 0.1

@export var volume_transition_duration := 0.1


@onready var _original_volume := db_to_linear(slide_loop.volume_db)


func _ready() -> void:
	slide_loop.volume_db = linear_to_db(0.0)
	slide_loop.play()
	await get_tree().process_frame


func _process(delta : float) -> void:
	if slip_pitch_curve == null or slip_volume_curve == null:
		return

	var curve_position := minf((slip - slip_min) / (slip_max - slip_min), 1.0)
	if curve_position < 0.0:
		slide_loop.stream_paused = true
	else:
		slide_loop.stream_paused = false

		var pitch_weight := slip_pitch_curve.sample_baked(curve_position)
		var pitch_scale := lerpf(slip_pitch_min, slip_pitch_max, pitch_weight)
		slide_loop.pitch_scale = move_toward(slide_loop.pitch_scale, pitch_scale, delta / pitch_transition_duration)

		var volume_weight := slip_volume_curve.sample_baked(curve_position)
		var target_volume_scale := lerpf(slip_volume_min, slip_volume_max, volume_weight)
		var previous_volume := db_to_linear(slide_loop.volume_db) / _original_volume
		var volume_scale := move_toward(previous_volume, target_volume_scale, delta / volume_transition_duration)
		slide_loop.volume_db = linear_to_db(volume_scale * _original_volume)
