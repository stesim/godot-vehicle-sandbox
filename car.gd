extends RigidBody3D


@onready var _front_left_wheel := %front_left_wheel
@onready var _front_right_wheel := %front_right_wheel
@onready var _rear_left_wheel := %rear_left_wheel
@onready var _rear_right_wheel := %rear_right_wheel
@onready var _wheels := [
	_front_left_wheel,
	_front_right_wheel,
	_rear_left_wheel,
	_rear_right_wheel,
]


func _integrate_forces(state : PhysicsDirectBodyState3D) -> void:
	for wheel in _wheels:
		wheel.process_physics(state)
