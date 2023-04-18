@tool
class_name Wheel
extends Node3D


enum Side {
	LEFT,
	RIGHT,
}


const WHEEL_RADIUS := 0.15


@export var side := Side.RIGHT :
	set(value):
		side = value
		if _hub:
			_hub.rotation.y = 0.0 if side == Side.RIGHT else PI

@export var is_powered := true

@export var is_steering := true

@export_range(0.0, 45.0, 0.01, "or_greater", "radians") var max_steering_angle := 20.0 / 180.0 * PI

@export var turning_center_offset := Vector2.ZERO

@export_range(0.0, 1.0, 0.01, "or_greater") var grip := 1.0

@export var grip_curve := Curve.new()

@export_range(0.0, 30.0, 0.01, "radians") var max_slip_angle := 20.0 / 180.0 * PI

@export var spring_strength := 200.0

@export_range(0.0, 1.0, 0.01, "or_greater") var spring_rest_length := 0.4 :
	set(value):
		spring_rest_length = value
		if _hub:
			_hub.position.y = -spring_rest_length

@export_range(0.0, 1.0, 0.01, "or_greater") var damping := 1.0

@export_range(0.0, 10.0, 0.01, "or_greater") var max_acceleration_force := 1.0


var _ray_cast_params := PhysicsRayQueryParameters3D.new()
var _forward_velocity := 0.0


@onready var _hub : Node3D = %hub
@onready var _mesh : Node3D = %mesh

@onready var _debug_suspension_force : DebugVectorOverlay = %debug_suspension_force
@onready var _debug_acceleration_force : DebugVectorOverlay = %debug_acceleration_force
@onready var _debug_grip_force : DebugVectorOverlay = %debug_grip_force


func _ready() -> void:
	side = side
	spring_rest_length = spring_rest_length

	set_process(not Engine.is_editor_hint())

	if get_parent() is CollisionObject3D:
		_ray_cast_params.exclude = [get_parent().get_rid()]

	grip_curve.bake()


func process_physics(vehicle_state : PhysicsDirectBodyState3D) -> void:
	_debug_suspension_force.vector = Vector3.ZERO
	_debug_acceleration_force.vector = Vector3.ZERO
	_debug_grip_force.vector = Vector3.ZERO

	_apply_steering()

	var ray_hit := _cast_ray(vehicle_state.get_space_state())
	var has_contact := not ray_hit.is_empty()

	if has_contact:
		_apply_suspension_force(vehicle_state, ray_hit.distance)
		_hub.position.y = -ray_hit.distance + WHEEL_RADIUS
	else:
		_hub.position.y = -spring_rest_length + WHEEL_RADIUS

	if has_contact:
		_apply_contact_forces(vehicle_state, ray_hit.normal)


func _process(delta : float) -> void:
	_update_mesh_rotation(delta)


func _apply_steering() -> void:
	if not is_steering:
		return

	var steering_input := Input.get_axis(&"steer_right", &"steer_left")
	var steering_angle := steering_input * max_steering_angle
	var turning_radius := turning_center_offset.y / tan(steering_angle)
	var ackermann_angle := atan(turning_center_offset.y / (turning_radius + turning_center_offset.x))
	rotation.y = ackermann_angle


func _apply_suspension_force(vehicle_state : PhysicsDirectBodyState3D, spring_length : float) -> void:
	var previous_spring_length := -_hub.position.y + WHEEL_RADIUS

	var spring_offset := spring_rest_length - spring_length
	var spring_force := spring_strength * spring_offset

	var spring_velocity := (spring_length - previous_spring_length) / vehicle_state.step
	var damping_force := damping * spring_velocity

	var total_force := (spring_force - damping_force) * global_transform.basis.y
	vehicle_state.apply_force(total_force, global_position - vehicle_state.transform.origin)

	_debug_suspension_force.vector = total_force


func _apply_contact_forces(vehicle_state : PhysicsDirectBodyState3D, contact_normal : Vector3) -> void:
	var vehicle_position := vehicle_state.transform.origin
	var contact_point := _hub.to_global(WHEEL_RADIUS * Vector3.DOWN)
	var forward_direction := contact_normal.cross(global_transform.basis.x).normalized()
	var slip_direction := contact_normal.cross(forward_direction).normalized()

	var velocity = vehicle_state.get_velocity_at_local_position(contact_point - vehicle_position)
	_forward_velocity = velocity.dot(forward_direction)
	var slip_velocity := velocity.dot(slip_direction)
	var virtual_mass_inv := 25.0 * vehicle_state.inverse_mass
	var slip_angle := absf(velocity.angle_to(forward_direction))
	var effective_grip := grip * grip_curve.sample_baked(minf(slip_angle, max_slip_angle) / max_slip_angle)
	var grip_force_magnitude := effective_grip * (-slip_velocity) / (vehicle_state.step * virtual_mass_inv)
	var grip_force := grip_force_magnitude * slip_direction
	_debug_grip_force.vector = grip_force

	var total_force := grip_force
	if is_powered:
		var acceleration_input := Input.get_axis(&"brake", &"accelerate")
		var acceleration_force := acceleration_input * max_acceleration_force * forward_direction
		total_force += acceleration_force
		_debug_acceleration_force.vector = acceleration_force

	vehicle_state.apply_force(total_force, contact_point - vehicle_position)


func _cast_ray(space_state : PhysicsDirectSpaceState3D) -> Dictionary:
	_ray_cast_params.from = global_position
	_ray_cast_params.to = global_position + (spring_rest_length + WHEEL_RADIUS) * Vector3.DOWN
	var result := space_state.intersect_ray(_ray_cast_params)
	if result.is_empty():
		return {}

	var local_hit_position := to_local(result.position)
	return {
		distance = -local_hit_position.y - WHEEL_RADIUS,
		normal = result.normal,
	}


func _update_mesh_rotation(delta : float) -> void:
	var direction := -1.0 if side == Side.RIGHT else 1.0
	var angular_velocity := _forward_velocity / WHEEL_RADIUS
	_mesh.rotate_x(delta * angular_velocity * direction)
