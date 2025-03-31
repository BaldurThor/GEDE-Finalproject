extends RigidBody3D

@onready var initial_position:= global_position

@export var initial_velocity:= Vector3.ZERO

func _ready() -> void:
	self.linear_velocity = initial_velocity
