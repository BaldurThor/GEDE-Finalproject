#include "ragdoll_bone_simulator_3d.h"
#include "godot_cpp/classes/physical_bone3d.hpp"
#include "godot_cpp/classes/skeleton3d.hpp"
#include "godot_cpp/core/math.hpp"
#include "godot_cpp/core/object.hpp"
#include "godot_cpp/variant/typed_array.hpp"
#include "godot_cpp/variant/utility_functions.hpp"
#include "godot_cpp/variant/vector3.hpp"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void RagdollBoneSimulator3D::_bind_methods() {
  ClassDB::bind_method(D_METHOD("get_bone_collision_layer"),
                       &RagdollBoneSimulator3D::get_bone_collision_layer);
  ClassDB::bind_method(
      D_METHOD("set_bone_collision_layer", "p_bone_collision_layer"),
      &RagdollBoneSimulator3D::set_bone_collision_layer);

  ClassDB::bind_method(D_METHOD("get_bone_collision_mask"),
                       &RagdollBoneSimulator3D::get_bone_collision_mask);
  ClassDB::bind_method(
      D_METHOD("set_bone_collision_mask", "p_bone_collision_mask"),
      &RagdollBoneSimulator3D::set_bone_collision_mask);

  ClassDB::bind_method(D_METHOD("get_collision_overwrite"),
                       &RagdollBoneSimulator3D::get_collision_overwrite);
  ClassDB::bind_method(
      D_METHOD("set_collision_overwrite", "p_collision_overwrite"),
      &RagdollBoneSimulator3D::set_collision_overwrite);

  ClassDB::bind_method(D_METHOD("get_angular_stiffness"),
                       &RagdollBoneSimulator3D::get_angular_stiffness);
  ClassDB::bind_method(D_METHOD("set_angular_stiffness", "p_angular_stiffness"),
                       &RagdollBoneSimulator3D::set_angular_stiffness);

  ClassDB::bind_method(D_METHOD("get_linear_stiffness"),
                       &RagdollBoneSimulator3D::get_linear_stiffness);
  ClassDB::bind_method(D_METHOD("set_linear_stiffness", "p_linear_stiffness"),
                       &RagdollBoneSimulator3D::set_linear_stiffness);

  ClassDB::bind_method(D_METHOD("get_angular_damping"),
                       &RagdollBoneSimulator3D::get_angular_damping);
  ClassDB::bind_method(D_METHOD("set_angular_damping", "p_angular_damping"),
                       &RagdollBoneSimulator3D::set_angular_damping);

  ClassDB::bind_method(D_METHOD("get_linear_damping"),
                       &RagdollBoneSimulator3D::get_linear_damping);
  ClassDB::bind_method(D_METHOD("set_linear_damping", "p_linear_damping"),
                       &RagdollBoneSimulator3D::set_linear_damping);

  ClassDB::bind_method(D_METHOD("get_maximum_force"),
                       &RagdollBoneSimulator3D::get_maximum_force);
  ClassDB::bind_method(D_METHOD("set_maximum_force", "p_maximum_force"),
                       &RagdollBoneSimulator3D::set_maximum_force);

  ClassDB::bind_method(D_METHOD("get_jerk_distance"),
                       &RagdollBoneSimulator3D::get_jerk_distance);
  ClassDB::bind_method(D_METHOD("set_jerk_distance", "p_jerk_distance"),
                       &RagdollBoneSimulator3D::set_jerk_distance);

  ClassDB::bind_method(D_METHOD("toggle_collapse"),
                       &RagdollBoneSimulator3D::toggle_collapse);
  ClassDB::bind_method(D_METHOD("set_collapsed", "p_collapsed"),
                       &RagdollBoneSimulator3D::set_collapsed);

  ClassDB::bind_method(D_METHOD("get_bone_x_angular_limit_upper"),
                       &RagdollBoneSimulator3D::get_bone_x_angular_limit_upper);
  ClassDB::bind_method(D_METHOD("set_bone_x_angular_limit_upper",
                                "p_bone_x_angular_limit_upper"),
                       &RagdollBoneSimulator3D::set_bone_x_angular_limit_upper);
  ClassDB::bind_method(D_METHOD("get_bone_x_angular_limit_lower"),
                       &RagdollBoneSimulator3D::get_bone_x_angular_limit_lower);
  ClassDB::bind_method(D_METHOD("set_bone_x_angular_limit_lower",
                                "p_bone_x_angular_limit_lower"),
                       &RagdollBoneSimulator3D::set_bone_x_angular_limit_lower);

  ClassDB::bind_method(D_METHOD("get_bone_y_angular_limit_upper"),
                       &RagdollBoneSimulator3D::get_bone_y_angular_limit_upper);
  ClassDB::bind_method(D_METHOD("set_bone_y_angular_limit_upper",
                                "p_bone_y_angular_limit_upper"),
                       &RagdollBoneSimulator3D::set_bone_y_angular_limit_upper);
  ClassDB::bind_method(D_METHOD("get_bone_y_angular_limit_lower"),
                       &RagdollBoneSimulator3D::get_bone_y_angular_limit_lower);
  ClassDB::bind_method(D_METHOD("set_bone_y_angular_limit_lower",
                                "p_bone_y_angular_limit_lower"),
                       &RagdollBoneSimulator3D::set_bone_y_angular_limit_lower);

  ClassDB::bind_method(D_METHOD("get_bone_z_angular_limit_upper"),
                       &RagdollBoneSimulator3D::get_bone_z_angular_limit_upper);
  ClassDB::bind_method(D_METHOD("set_bone_z_angular_limit_upper",
                                "p_bone_z_angular_limit_upper"),
                       &RagdollBoneSimulator3D::set_bone_z_angular_limit_upper);
  ClassDB::bind_method(D_METHOD("get_bone_z_angular_limit_lower"),
                       &RagdollBoneSimulator3D::get_bone_z_angular_limit_lower);
  ClassDB::bind_method(D_METHOD("set_bone_z_angular_limit_lower",
                                "p_bone_z_angular_limit_lower"),
                       &RagdollBoneSimulator3D::set_bone_z_angular_limit_lower);
  ClassDB::bind_method(D_METHOD("get_angular_limit_overwrite"),
                       &RagdollBoneSimulator3D::get_angular_limit_overwrite);
  ClassDB::bind_method(
      D_METHOD("set_angular_limit_overwrite", "p_angular_limit_overwrite"),
      &RagdollBoneSimulator3D::set_angular_limit_overwrite);

  ADD_GROUP("Collision", "collision_");
  ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_bone_layer",
                            PROPERTY_HINT_LAYERS_3D_PHYSICS),
               "set_bone_collision_layer", "get_bone_collision_layer");
  ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_bone_mask",
                            PROPERTY_HINT_LAYERS_3D_PHYSICS),
               "set_bone_collision_mask", "get_bone_collision_mask");
  ADD_PROPERTY(PropertyInfo(Variant::BOOL, "collision_overwrite"),
               "set_collision_overwrite", "get_collision_overwrite");

  ADD_GROUP("Linear", "linear_");
  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "linear_stiffness",
                            PROPERTY_HINT_RANGE, "0,100,0.01"),
               "set_linear_stiffness", "get_linear_stiffness");
  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "linear_damping",
                            PROPERTY_HINT_RANGE, "0,100,0.01"),
               "set_linear_damping", "get_linear_damping");

  ADD_GROUP("Angular", "angular_");
  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "angular_stiffness",
                            PROPERTY_HINT_RANGE, "0,100,0.01"),
               "set_angular_stiffness", "get_angular_stiffness");
  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "angular_damping",
                            PROPERTY_HINT_RANGE, "0,100,0.01"),
               "set_angular_damping", "get_angular_damping");

  ADD_GROUP("Angular Limits", "angular_limit_");
  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "angular_limit_bone_x_upper",
                            PROPERTY_HINT_RANGE, "-180,180,0.01"),
               "set_bone_x_angular_limit_upper",
               "get_bone_x_angular_limit_upper");
  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "angular_limit_bone_x_lower",
                            PROPERTY_HINT_RANGE, "-180,180,0.01"),
               "set_bone_x_angular_limit_lower",
               "get_bone_x_angular_limit_lower");
  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "angular_limit_bone_y_upper",
                            PROPERTY_HINT_RANGE, "-180,180,0.01"),
               "set_bone_y_angular_limit_upper",
               "get_bone_y_angular_limit_upper");
  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "angular_limit_bone_y_lower",
                            PROPERTY_HINT_RANGE, "-180,180,0.01"),
               "set_bone_y_angular_limit_lower",
               "get_bone_y_angular_limit_lower");
  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "angular_limit_bone_z_upper",
                            PROPERTY_HINT_RANGE, "-180,180,0.01"),
               "set_bone_z_angular_limit_upper",
               "get_bone_z_angular_limit_upper");
  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "angular_limit_bone_z_lower",
                            PROPERTY_HINT_RANGE, "-180,180,0.01"),
               "set_bone_z_angular_limit_lower",
               "get_bone_z_angular_limit_lower");
  ADD_PROPERTY(PropertyInfo(Variant::BOOL, "angular_limit_overwrite"),
               "set_angular_limit_overwrite", "get_angular_limit_overwrite");

  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "maximum_force",
                            PROPERTY_HINT_RANGE, "0,100,0.01"),
               "set_maximum_force", "get_maximum_force");

  ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "jerk_distance",
                            PROPERTY_HINT_RANGE, "0,100,0.01"),
               "set_jerk_distance", "get_jerk_distance");
}

RagdollBoneSimulator3D::RagdollBoneSimulator3D() {
  _engine = godot::Engine::get_singleton();
  bone_collision_layer = 1;
  bone_collision_mask = 1;
  collision_overwrite = false;

  angular_stiffness = 1.0;
  linear_stiffness = 1.0;
  angular_damping = 1.0;
  linear_damping = 1.0;
  maximum_force = 1.0;
  jerk_distance = 1.0;

  collapsed = false;
  physical_bones = {};

  bone_x_angular_limit_upper = 45.0;
  bone_x_angular_limit_lower = -45.0;

  bone_y_angular_limit_upper = 45.0;
  bone_y_angular_limit_lower = -45.0;

  bone_z_angular_limit_upper = 45.0;
  bone_z_angular_limit_lower = -45.0;

  angular_limit_overwrite = false;
}

RagdollBoneSimulator3D::~RagdollBoneSimulator3D() {}

void RagdollBoneSimulator3D::ready() {
  TypedArray<Node> children = get_children();
  for (int i = 0; i < children.size(); i++) {
    Node *child = Object::cast_to<Node>(children[i]);
    if (child->get_class() == "PhysicalBone3D") {
      PhysicalBone3D *bone = Object::cast_to<PhysicalBone3D>(children[i]);
      if (angular_limit_overwrite) {
        bone->set_joint_type(PhysicalBone3D::JOINT_TYPE_6DOF);
        bone->set("joint_constraints/x/angular_limit_upper",
                  bone_x_angular_limit_upper);
        bone->set("joint_constraints/x/angular_limit_lower",
                  bone_x_angular_limit_lower);
        bone->set("joint_constraints/y/angular_limit_upper",
                  bone_y_angular_limit_upper);
        bone->set("joint_constraints/y/angular_limit_lower",
                  bone_y_angular_limit_lower);
        bone->set("joint_constraints/z/angular_limit_upper",
                  bone_z_angular_limit_upper);
        bone->set("joint_constraints/z/angular_limit_lower",
                  bone_z_angular_limit_lower);
      }
      if (collision_overwrite) {
        bone->set_collision_layer(bone_collision_layer);
        bone->set_collision_mask(bone_collision_mask);
      }
      bone->set_friction(0.0);
      bone->set_gravity_scale(0.01);
      physical_bones.push_back(bone);
    }
  }
  physical_bones_start_simulation();
  set_physics_process(true);
}

void RagdollBoneSimulator3D::physics_process(double delta) {
  if (!collapsed) {
    if (!get_skeleton()) {
      return; // No skeleton, nothing todo
    }
    Skeleton3D *target_skeleton = Object::cast_to<Skeleton3D>(get_skeleton());
    for (int i = 0; i < physical_bones.size(); i++) {
      PhysicalBone3D *bone = physical_bones[i];
      Transform3D target_transform =
          target_skeleton->get_global_transform() *
          target_skeleton->get_bone_global_pose(bone->get_bone_id());
      Transform3D current_transform =
          bone->get_global_transform() * bone->get_body_offset().inverse();

      Vector3 position_difference =
          target_transform.origin - current_transform.origin;

      if (jerk_distance > 0.0 &&
          (position_difference.length() > jerk_distance)) {
        bone->set_global_transform(target_transform * bone->get_body_offset());
      } else {
        Vector3 linear_velocity = bone->get_linear_velocity();
        Vector3 linear_force = hookes_law(position_difference, linear_velocity,
                                          linear_stiffness, linear_damping);
        linear_velocity += linear_force.limit_length(maximum_force) * delta *
                           _engine->get_physics_ticks_per_second();
        bone->set_linear_velocity(linear_velocity);

        Basis rotation_difference =
            (target_transform.basis * current_transform.basis.inverse());

        Vector3 angular_velocity = bone->get_angular_velocity();
        Vector3 angular_force =
            hookes_law(rotation_difference.get_euler(), angular_velocity,
                       angular_stiffness, angular_damping);
        angular_velocity += angular_force.limit_length(maximum_force) * delta *
                            _engine->get_physics_ticks_per_second();
        bone->set_angular_velocity(angular_velocity);
      }
    }
  }
}

void RagdollBoneSimulator3D::_notification(int p_what) {
  switch (p_what) {
  case NOTIFICATION_READY:
    ready();
    break;
  case NOTIFICATION_PHYSICS_PROCESS:
    physics_process(get_physics_process_delta_time());
    break;
  }
}

void RagdollBoneSimulator3D::set_bone_collision_layer(
    const int p_bone_collision_layer) {
  bone_collision_layer = p_bone_collision_layer;
}
int RagdollBoneSimulator3D::get_bone_collision_layer() const {
  return bone_collision_layer;
}

void RagdollBoneSimulator3D::set_bone_collision_mask(
    const int p_bone_collision_mask) {
  bone_collision_mask = p_bone_collision_mask;
}
int RagdollBoneSimulator3D::get_bone_collision_mask() const {
  return bone_collision_mask;
}

void RagdollBoneSimulator3D::set_collision_overwrite(
    const bool p_collision_overwrite) {
  collision_overwrite = p_collision_overwrite;
}
bool RagdollBoneSimulator3D::get_collision_overwrite() const {
  return collision_overwrite;
}

void RagdollBoneSimulator3D::set_angular_stiffness(
    const double p_angular_stiffness) {
  angular_stiffness = p_angular_stiffness;
}
double RagdollBoneSimulator3D::get_angular_stiffness() const {
  return angular_stiffness;
}

void RagdollBoneSimulator3D::set_linear_stiffness(
    const double p_linear_stiffness) {
  linear_stiffness = p_linear_stiffness;
}
double RagdollBoneSimulator3D::get_linear_stiffness() const {
  return linear_stiffness;
}

void RagdollBoneSimulator3D::set_angular_damping(
    const double p_angular_damping) {
  angular_damping = p_angular_damping;
}
double RagdollBoneSimulator3D::get_angular_damping() const {
  return angular_damping;
}

void RagdollBoneSimulator3D::set_linear_damping(const double p_linear_damping) {
  linear_damping = p_linear_damping;
}
double RagdollBoneSimulator3D::get_linear_damping() const {
  return linear_damping;
}

void RagdollBoneSimulator3D::set_maximum_force(const double p_maximum_force) {
  maximum_force = p_maximum_force;
}
double RagdollBoneSimulator3D::get_maximum_force() const {
  return maximum_force;
}

void RagdollBoneSimulator3D::set_jerk_distance(const double p_jerk_distance) {
  jerk_distance = p_jerk_distance;
}
double RagdollBoneSimulator3D::get_jerk_distance() const {
  return jerk_distance;
}

void RagdollBoneSimulator3D::toggle_collapse() { set_collapsed(!collapsed); }
void RagdollBoneSimulator3D::set_collapsed(bool p_collapsed) {
  if (collapsed == p_collapsed) {
    return;
  }
  double gravity_scale = 1.0;
  if (collapsed) {
    gravity_scale = 0.01;
  }
  collapsed = p_collapsed;
  for (int i = 0; i < physical_bones.size(); i++) {
    PhysicalBone3D *bone = physical_bones[i];
    bone->set_gravity_scale(gravity_scale);
  }
}

Vector3 RagdollBoneSimulator3D::hookes_law(Vector3 displacement,
                                           Vector3 current_velocity,
                                           double stiffness, double damping) {
  damping = Math::clamp(damping, 0.0, stiffness);
  return (stiffness * displacement) - (damping * current_velocity);
}

void RagdollBoneSimulator3D::set_bone_x_angular_limit_upper(
    const double p_bone_x_angular_limit_upper) {
  bone_x_angular_limit_upper = p_bone_x_angular_limit_upper;
}
double RagdollBoneSimulator3D::get_bone_x_angular_limit_upper() const {
  return bone_x_angular_limit_upper;
}

void RagdollBoneSimulator3D::set_bone_x_angular_limit_lower(
    const double p_bone_x_angular_limit_lower) {
  bone_x_angular_limit_lower = p_bone_x_angular_limit_lower;
}
double RagdollBoneSimulator3D::get_bone_x_angular_limit_lower() const {
  return bone_x_angular_limit_lower;
}

void RagdollBoneSimulator3D::set_bone_y_angular_limit_upper(
    const double p_bone_y_angular_limit_upper) {
  bone_y_angular_limit_upper = p_bone_y_angular_limit_upper;
}
double RagdollBoneSimulator3D::get_bone_y_angular_limit_upper() const {
  return bone_y_angular_limit_upper;
}

void RagdollBoneSimulator3D::set_bone_y_angular_limit_lower(
    const double p_bone_y_angular_limit_lower) {
  bone_y_angular_limit_lower = p_bone_y_angular_limit_lower;
}
double RagdollBoneSimulator3D::get_bone_y_angular_limit_lower() const {
  return bone_y_angular_limit_lower;
}

void RagdollBoneSimulator3D::set_bone_z_angular_limit_upper(
    const double p_bone_z_angular_limit_upper) {
  bone_z_angular_limit_upper = p_bone_z_angular_limit_upper;
}
double RagdollBoneSimulator3D::get_bone_z_angular_limit_upper() const {
  return bone_z_angular_limit_upper;
}

void RagdollBoneSimulator3D::set_bone_z_angular_limit_lower(
    const double p_bone_z_angular_limit_lower) {
  bone_z_angular_limit_lower = p_bone_z_angular_limit_lower;
}
double RagdollBoneSimulator3D::get_bone_z_angular_limit_lower() const {
  return bone_z_angular_limit_lower;
}

void RagdollBoneSimulator3D::set_angular_limit_overwrite(
    const bool p_angular_limit_overwrite) {
  angular_limit_overwrite = p_angular_limit_overwrite;
}
bool RagdollBoneSimulator3D::get_angular_limit_overwrite() const {
  return angular_limit_overwrite;
}
