class_name AntiRollBar
extends Resource


@export var stiffness := 30000.0


@export_subgroup("Input")

@export var average_travel := 0.0


func calculate_force(suspension_length : float) -> float:
	var displacement := suspension_length - average_travel
	return -stiffness * displacement
