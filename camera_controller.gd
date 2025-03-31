extends Node3D

#this signal is so the main player "class" can rotate the players movement according to the cameras direction.
signal set_cam_rotation(_cam_rotation : float)
@export var player : CharacterBody3D

@onready var camera: Camera3D = %Camera
@onready var cam_yaw: Node3D = %CamYaw
@onready var cam_pitch: Node3D = %CamPitch
@onready var spring_arm: SpringArm3D = %SpringArm
@onready var springy_camera: Node3D = %SpringyCamera
@onready var camera_reset_timeout: Timer = %CameraResetTimeout

@export var mouse_sensitivity: float = 0.7

var yaw: float = 0.0
var pitch: float = 0.0
var yaw_acceleration: float = 15.0
var pitch_acceleration: float = 15.0
var pitch_max: float = 50.0
var pitch_min: float = -75.0
var target_pitch: float = 0.0

var zoom: float = 2.0
var zoom_max: float = 10.0
var zoom_min: float = 0.5

var springy_camera_multiplier: float = -0.05
var springy_camera_acceleration: float = 5.0

var camera_reset_acceleration: float = 0.1

#fetches them from the editor so they can be visually changed there.
var spring_arm_length: float = 0.0
var camera_distance: float = 0.0

var movement_direction: Vector3 = Vector3.ZERO

var top_down_rotation: bool = false
var select_pressed: bool = false
var select_pressed_timer: float = 0.0
var select_drag_timer: float = 0.1
var drag_started: bool = false

# click is where it was when the click happened
# top down is where it currently is (if relevant)
# -1 on y axis means that no click is performed, we use that because we only care about the x and z axis
# and it can never be negative unless out of bounds
var RESET_MOUSE_POS: Vector3 = Vector3(0, -1, 0)

var top_down_position_overlay: Vector2 = Vector2.ZERO
var click_position_overlay: Vector2 = Vector2.ZERO
var select_reset: bool = false


func _ready():
	#spring_arm.add_excluded_object(player.get_rid())
	spring_arm_length = spring_arm.spring_length
	camera_distance = camera.position.z
	pitch = remap(zoom, zoom_min, zoom_max, 0.0, pitch_min)
	target_pitch = pitch
	cam_pitch.rotation_degrees.x = pitch


func _input(event):
	# Action cam!
	if event is InputEventMouseMotion:
		camera_reset_timeout.start()
		yaw += -event.relative.x * mouse_sensitivity
		pitch += -event.relative.y * mouse_sensitivity
		pitch = clampf(pitch, pitch_min, pitch_max)
		target_pitch = pitch
	elif event.is_action_pressed("zoom_out"):
		zoom_out()
	elif event.is_action_pressed("zoom_in"):
		zoom_in()

func _physics_process(delta):
	if not target_pitch == pitch:
		pitch = lerp(pitch, target_pitch, camera_reset_acceleration * delta)

	springy_camera.position = lerp(springy_camera.position, movement_direction * springy_camera_multiplier, springy_camera_acceleration * delta)

	spring_arm.spring_length = spring_arm_length * zoom
	camera.position.z = camera_distance * zoom

	cam_yaw.rotation_degrees.y = lerp(cam_yaw.rotation_degrees.y, yaw, yaw_acceleration * delta)

	cam_pitch.rotation_degrees.x = lerp(cam_pitch.rotation_degrees.x, pitch, pitch_acceleration * delta)

	set_cam_rotation.emit(cam_yaw.rotation.y)
	

func zoom_in() -> void:
	camera_reset_timeout.start()
	zoom -= zoom * 0.2
	zoom = clampf(zoom, zoom_min, zoom_max)


func zoom_out() -> void:
	camera_reset_timeout.start()
	zoom += zoom * 0.25
	zoom = clampf(zoom, zoom_min, zoom_max)
