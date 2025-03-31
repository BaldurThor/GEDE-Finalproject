extends CharacterBody3D

var target_velo: Vector3 = Vector3.ZERO
@onready var animation_tree: AnimationTree = $AnimationTree

# --- INVERSE KINEMATICS ---
@onready var skeleton: Skeleton3D = $Armature/Skeleton3D
#feet
@onready var left_foot_ray: RayCast3D = skeleton.get_node("left_foot_ray")
@onready var right_foot_ray: RayCast3D = skeleton.get_node("right_foot_ray")

@onready var left_foot_target: Marker3D = skeleton.get_node("left_foot_target")
@onready var right_foot_target: Marker3D = skeleton.get_node("right_foot_target")

@onready var left_foot_ik: SkeletonIK3D = skeleton.get_node("left_foot_ik")
@onready var right_foot_ik: SkeletonIK3D = skeleton.get_node("right_foot_ik")

#hand
@onready var left_hand_cast: ShapeCast3D = skeleton.get_node("left_hand_cast")
@onready var right_hand_cast: ShapeCast3D = skeleton.get_node("right_hand_cast")

@onready var left_hand_target: Marker3D = skeleton.get_node("left_hand_target")
@onready var right_hand_target: Marker3D = skeleton.get_node("right_hand_target")

@onready var left_hand_ik: SkeletonIK3D = skeleton.get_node("left_hand_ik")
@onready var right_hand_ik: SkeletonIK3D = skeleton.get_node("right_hand_ik")

var is_idle: bool = false
# --------------------------------------------------------------------------

var cam_rotation: float = 0.0


func _ready() -> void:
	floor_snap_length = 0.5
	floor_block_on_wall = true
	floor_max_angle = deg_to_rad(60.0)

func _physics_process(delta: float) -> void:
	target_velo = Vector3.ZERO
	
	if Input.is_action_pressed("forward"):
		target_velo.z += 1
	if Input.is_action_pressed("back"):
		target_velo.z -= 1
	if Input.is_action_pressed("left"):
		target_velo.x += 1
	if Input.is_action_pressed("right"):
		target_velo.x -= 1
	if Input.is_action_just_pressed("lmb"):
		%RagdollBoneSimulator3D.toggle_collapse()
	animation_tree['parameters/blend_position'].x = -target_velo.x
	animation_tree['parameters/blend_position'].y = target_velo.z
	
	target_velo = target_velo.rotated(Vector3(0,1,0), cam_rotation)
	
	if target_velo.length_squared() > 0.01:
		$Armature.look_at(global_position + Vector3.FORWARD.rotated(Vector3.UP, cam_rotation))
	
	target_velo *= 2
	
	target_velo.y = velocity.y
	if not is_on_floor():
		target_velo.y += get_gravity().y * delta
	
	velocity = target_velo
	move_and_slide()
	is_idle = target_velo.length_squared() < 0.01 and is_on_floor()
	
	# --- Inverse Kinematics ---
	update_foot_ik(left_foot_ray, left_foot_target, left_foot_ik, delta)
	update_foot_ik(right_foot_ray, right_foot_target, right_foot_ik, delta)
	
	update_hand_ik(left_hand_cast, left_hand_target, left_hand_ik, delta)
	update_hand_ik(right_hand_cast, right_hand_target, right_hand_ik, delta)


func _on_camera_root_set_cam_rotation(_cam_rotation: float) -> void:
	cam_rotation = _cam_rotation


# --- INVERSE KINEMATICS ---
func update_foot_ik(ray: RayCast3D, target: Marker3D, ik: SkeletonIK3D, _delta: float) -> void:
	if is_idle and ray.is_colliding():
		var hit_pos: Vector3 = ray.get_collision_point()
		var normal: Vector3 = ray.get_collision_normal()

		#snap the foot instantly and only adjust to Y (height)
		var current_pos: Vector3 = target.global_position
		var snapped_pos: Vector3 = Vector3(current_pos.x, hit_pos.y + normal.y * 0.05, current_pos.z)
		target.global_position = snapped_pos

		#alignto slope
		var character_forward: Vector3 = -global_transform.basis.z
		var forward: Vector3 = character_forward.slide(normal).normalized()
		var right: Vector3 = normal.cross(forward).normalized()
		forward = right.cross(normal).normalized()

		var foot_basis: Basis = Basis()
		foot_basis.x = right
		foot_basis.y = normal
		foot_basis.z = forward
		target.global_transform.basis = foot_basis

		ik.start()
		ik.influence = 1.0
	else:
		ik.influence = 0.0
		ik.stop()
		
		
func update_hand_ik(cast: ShapeCast3D, target: Marker3D, ik: SkeletonIK3D, _delta: float) -> void:
	cast.force_update_transform()

	if cast.is_colliding():
		var hit_pos: Vector3 = cast.get_collision_point(0)
		var normal: Vector3 = cast.get_collision_normal(0)

		var target_pos: Vector3 = hit_pos + normal * 0.05
		target.global_position = target_pos

		var forward: Vector3 = -normal
		var up: Vector3 = Vector3.UP
		var right: Vector3 = up.cross(forward).normalized()
		up = forward.cross(right).normalized()

		var n_basis: Basis = Basis()
		n_basis.x = right
		n_basis.y = up
		n_basis.z = forward
		target.global_transform.basis = n_basis

		ik.start()
		ik.influence = 1.0
	else:
		ik.influence = 0.0
		ik.stop()
