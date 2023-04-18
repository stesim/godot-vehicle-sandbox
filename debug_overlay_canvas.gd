class_name DebugOverlayCanvas
extends Control


func _process(_delta : float) -> void:
	queue_redraw()


func _draw() -> void:
	for overlay in get_tree().get_nodes_in_group(&"debug_overlays"):
		overlay.draw(self)
