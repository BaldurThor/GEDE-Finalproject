#ifndef RAGDOLL_BONE_SIMULATOR_H
#define RAGDOLL_BONE_SIMULATOR_H

#include "godot_cpp/classes/physical_bone3d.hpp"
#include "godot_cpp/variant/vector3.hpp"
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/physical_bone_simulator3d.hpp>
#include <vector>

namespace godot {

class RagdollBoneSimulator3D : public PhysicalBoneSimulator3D {
  GDCLASS(RagdollBoneSimulator3D, PhysicalBoneSimulator3D)

private:
  Engine *_engine;
  int bone_collision_layer;
  int bone_collision_mask;
  bool collision_overwrite;

  double angular_stiffness;
  double linear_stiffness;
  double angular_damping;
  double linear_damping;
  double maximum_force;
  double jerk_distance;

  double bone_x_angular_limit_upper;
  double bone_x_angular_limit_lower;

  double bone_y_angular_limit_upper;
  double bone_y_angular_limit_lower;

  double bone_z_angular_limit_upper;
  double bone_z_angular_limit_lower;

  bool angular_limit_overwrite;

  bool collapsed;
  std::vector<PhysicalBone3D *> physical_bones;

  Vector3 hookes_law(Vector3 displacement, Vector3 current_velocity,
                     double stiffness, double damping);

protected:
  static void _bind_methods();

public:
  RagdollBoneSimulator3D();
  ~RagdollBoneSimulator3D();

  void ready();
  void physics_process(double delta);

  void _notification(int p_what);

  void set_bone_collision_layer(const int p_bone_collision_layer);
  int get_bone_collision_layer() const;

  void set_bone_collision_mask(const int p_bone_collision_mask);
  int get_bone_collision_mask() const;

  void set_collision_overwrite(const bool p_collision_overwrite);
  bool get_collision_overwrite() const;

  void set_angular_stiffness(const double p_angular_stiffness);
  double get_angular_stiffness() const;

  void set_linear_stiffness(const double p_linear_stiffness);
  double get_linear_stiffness() const;

  void set_angular_damping(const double p_angular_damping);
  double get_angular_damping() const;

  void set_linear_damping(const double p_linear_damping);
  double get_linear_damping() const;

  void set_maximum_force(const double p_maximum_force);
  double get_maximum_force() const;

  void set_jerk_distance(const double p_jerk_distance);
  double get_jerk_distance() const;

  void toggle_collapse();
  void set_collapsed(bool p_collapsed);

  void
  set_bone_x_angular_limit_upper(const double p_bone_x_angular_limit_upper);
  double get_bone_x_angular_limit_upper() const;

  void
  set_bone_x_angular_limit_lower(const double p_bone_x_angular_limit_lower);
  double get_bone_x_angular_limit_lower() const;

  void
  set_bone_y_angular_limit_upper(const double p_bone_y_angular_limit_upper);
  double get_bone_y_angular_limit_upper() const;

  void
  set_bone_y_angular_limit_lower(const double p_bone_y_angular_limit_lower);
  double get_bone_y_angular_limit_lower() const;

  void
  set_bone_z_angular_limit_upper(const double p_bone_z_angular_limit_upper);
  double get_bone_z_angular_limit_upper() const;

  void
  set_bone_z_angular_limit_lower(const double p_bone_z_angular_limit_lower);
  double get_bone_z_angular_limit_lower() const;

  void set_angular_limit_overwrite(const bool p_angular_limit_overwrite);
  bool get_angular_limit_overwrite() const;
};

} // namespace godot

#endif // RAGDOLL_BONE_SIMULATOR_H
