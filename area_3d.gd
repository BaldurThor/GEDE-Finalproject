extends Area3D


func _on_body_entered(body: Node3D) -> void:
	body.global_position = body.initial_position
	body.linear_velocity = body.initial_velocity
