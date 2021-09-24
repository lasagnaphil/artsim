//
// Created by lasagnaphil on 8/21/21.
//

#ifndef ARTSIM_ART_BODY_H
#define ARTSIM_ART_BODY_H

#include <artsim/types.h>
#include <artsim/rigid_body.h>
#include <artsim/math/dynmat.h>
#include <artsim/anim/pose_tree.h>
#include <artsim/collision/collision_shape.h>

#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

namespace artsim {

struct Material;
struct World;

enum JointType : int {
    JOINT_TYPE_REVOLUTE_X = 0,
    JOINT_TYPE_REVOLUTE_Y,
    JOINT_TYPE_REVOLUTE_Z,

    JOINT_TYPE_PRISMATIC_X,
    JOINT_TYPE_PRISMATIC_Y,
    JOINT_TYPE_PRISMATIC_Z,

    JOINT_TYPE_SPHERICAL,
    JOINT_TYPE_FLOATING,
};

#define JOINT_REVOLUTE_CASE case JOINT_TYPE_REVOLUTE_X: case JOINT_TYPE_REVOLUTE_Y: case JOINT_TYPE_REVOLUTE_Z:

#define JOINT_PRISMATIC_CASE case JOINT_TYPE_PRISMATIC_X: case JOINT_TYPE_PRISMATIC_Y: case JOINT_TYPE_PRISMATIC_Z:

#define JOINT_DOF_1_CASE case JOINT_TYPE_REVOLUTE_X: case JOINT_TYPE_REVOLUTE_Y: case JOINT_TYPE_REVOLUTE_Z: \
case JOINT_TYPE_PRISMATIC_X: case JOINT_TYPE_PRISMATIC_Y: case JOINT_TYPE_PRISMATIC_Z:

struct Joint {
    constexpr static real default_kp = 0.0;
    constexpr static real default_kd = 0.0;
    constexpr static real default_maxvel = 100.0;

    JointType type;
    bool limit_enabled;
    real limit_min;
    real limit_max;
    real kp, kd;
    real max_velocity;

    static Joint floating(real kp = 0.0, real kd = 0.0, real max_velocity = default_maxvel) {
        return {JOINT_TYPE_FLOATING, false, 0, 0, kp, kd, max_velocity};
    }
    static Joint revolute_x(real kp = default_kp, real kd = default_kd, real max_velocity = default_maxvel) {
        return {JOINT_TYPE_REVOLUTE_X, false, 0, 0, kp, kd, max_velocity};
    }
    static Joint revolute_y(real kp = default_kp, real kd = default_kd, real max_velocity = default_maxvel) {
        return {JOINT_TYPE_REVOLUTE_Y, false, 0, 0, kp, kd, max_velocity};
    }
    static Joint revolute_z(real kp = default_kp, real kd = default_kd, real max_velocity = default_maxvel) {
        return {JOINT_TYPE_REVOLUTE_Z, false, 0, 0, kp, kd, max_velocity};
    }
    static Joint prismatic_x(real kp = default_kp, real kd = default_kd, real max_velocity = default_maxvel) {
        return {JOINT_TYPE_PRISMATIC_X, false, 0, 0, kp, kd, max_velocity};
    }
    static Joint prismatic_y(real kp = default_kp, real kd = default_kd, real max_velocity = default_maxvel) {
        return {JOINT_TYPE_PRISMATIC_Y, false, 0, 0, kp, kd, max_velocity};
    }
    static Joint prismatic_z(real kp = default_kp, real kd = default_kd, real max_velocity = default_maxvel) {
        return {JOINT_TYPE_PRISMATIC_Z, false, 0, 0, kp, kd, max_velocity};
    }
    static Joint spherical(real kp = default_kp, real kd = default_kd, real max_velocity = default_maxvel) {
        return {JOINT_TYPE_SPHERICAL, false, 0, 0, kp, kd, max_velocity};
    }

    void enable_limit(real limit_min, real limit_max) {
        limit_enabled = true;
        this->limit_min = limit_min;
        this->limit_max = limit_max;
    }

    void disable_limit() {
        limit_enabled = false;
    }

    int pos_dof() {
        switch (type) {
            JOINT_DOF_1_CASE { return 1; }
            case JOINT_TYPE_SPHERICAL: return 4;
            case JOINT_TYPE_FLOATING: return 7;
            default: return 0;
        }
    }
    int vel_dof() {
        switch (type) {
            JOINT_DOF_1_CASE { return 1; }
            case JOINT_TYPE_SPHERICAL: return 3;
            case JOINT_TYPE_FLOATING: return 6;
            default: return 0;
        }
    }
};

struct Link {
    glmx::tsmat3x3<real> inertia; // inertia from link frame
    glmx::tspmat<real> I_j; // Spatial mass matrix from joint frame
    real density;
    real mass;
    CollisionShape col_shape;
    glmx::rquat_transform local_joint_pose;
    glmx::rquat_transform local_link_pose;
    int parent_idx;
    Id<Material> mat_id;
    std::string obj_filename;

    static Link create(CollisionShape col_shape, real density,
                       glmx::ttransform<real> local_joint_pose, glmx::ttransform<real> local_link_pose,
                       int parent_idx, Id<Material> mat_id, std::string obj_filename = "");

    static Link create(CollisionShape col_shape, real mass, glmx::tsmat3x3<real> inertia,
                       glmx::ttransform<real> local_joint_pose, glmx::ttransform<real> local_link_pose,
                       int parent_idx, Id<Material> mat_id, std::string obj_filename = "");
};

struct ArticulatedBodySpec {
    std::vector<std::string> names;
    std::vector<Link> links;
    std::vector<Joint> joints;

    std::vector<float> initial_state;

    bool floating = false;

    std::vector<int> joint_pos_dofs;
    std::vector<int> joint_pos_dof_starts;
    std::vector<int> joint_vel_dofs;
    std::vector<int> joint_vel_dof_starts;
    int num_pos_dofs = 0;
    int num_vel_dofs = 0;

    std::vector<int> parents;
    std::vector<int> children_buffer;
    std::vector<int> children_buffer_starts;
    std::vector<int> bfs_iteration_order;

    bool build_finished = false;

    ArticulatedBodySpec() = default;

    void add_link_and_joint(Link link, Joint joint, const std::string& name = "");

    void build();

    int get_num_joints() const { return joints.size(); }
    int get_num_links() const { return links.size(); }

    int get_num_pos_dofs() const { return num_pos_dofs; };
    int get_num_vel_dofs() const { return num_vel_dofs; };

    int get_num_children(int joint_idx) const {
        return children_buffer_starts[joint_idx+1] - children_buffer_starts[joint_idx];
    }
    const int * get_children(int joint_idx) const {
        return &children_buffer[children_buffer_starts[joint_idx]];
    }

    int get_index(const char* name) const;

    void scale_link(int link_idx, const glm::rvec3& scale, bool scale_shapes);
    void scale_link(int link_idx, const glm::rmat3& rot, const glm::rvec3& scale);

    PoseTree get_pose_tree();
};

class ArticulatedBody {
private:
    World* world;

    Id<ArticulatedBodySpec> spec_id;

    CollisionFlags collision_flags;

    std::vector<Id<RigidBody>> bodies;

    int num_pos_dofs;
    int num_vel_dofs;

    std::vector<real> q;
    std::vector<real> u;
    std::vector<real> udot;
    std::vector<real> tau;
    std::vector<real> q_target;

public:
    void init(World* world, Id<ArticulatedBodySpec> art_spec_id);

    void reset();
    void randomize_positions();

    int get_num_pos_dofs() const { return num_pos_dofs; }
    int get_num_vel_dofs() const { return num_vel_dofs; }
    int get_num_joints() const { return bodies.size(); }
    int get_num_links() const { return bodies.size(); }

    CollisionFlags get_collision_flags() const { return collision_flags; }

    real* get_pos_buf() { return q.data(); }
    real* get_vel_buf() { return u.data(); }
    real* get_acc_buf() { return udot.data(); }
    real* get_internal_force_buf() { return tau.data(); }
    real* get_target_pos_buf() { return q_target.data(); }

    real get_joint_pos_1dof(int joint_idx) const;
    glm::rquat get_joint_pos_spherical(int joint_idx) const;
    glmx::rtransform get_root_transform() const;
    real get_joint_vel_1dof(int joint_idx) const;
    glm::rvec3 get_joint_vel_spherical(int joint_idx) const;
    glmx::rscrew get_root_vel() const;

    void set_joint_pos_1dof(int joint_idx, real qj);
    void set_joint_pos_spherical(int joint_idx, const glm::rquat& qj);
    void set_root_transform(const glmx::rtransform& rootT);
    void set_joint_vel_1dof(int joint_idx, real qj);
    void set_joint_vel_spherical(int joint_idx, const glm::rvec3& qj);
    void set_root_vel(const glmx::rscrew& V);

    void forward_kinematics();

    void update_colliders();

    void forward_dynamics(const glm::rvec3& gravity, real dt);
    void forward_dynamics_with_contact(const glm::rvec3& gravity, real dt);
    void integrate(real dt);
    void simulate(const glm::rvec3& gravity, real dt);

    void mass_matrix(OUT glmx::dynmat_view<real> M, real dt = 0);
    void multiply_inverse_mass_matrix(glmx::dynmat_view<real> X, OUT glmx::dynmat_view<real> Minv_X, real dt = 0);

    glm::rvec3 get_center_of_mass() const;
};

}

#endif //ARTSIM_ART_BODY_H
