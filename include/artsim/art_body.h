//
// Created by lasagnaphil on 8/21/21.
//

#ifndef ARTSIM_ART_BODY_H
#define ARTSIM_ART_BODY_H

#include <artsim/types.h>
#include <artsim/collision_shape.h>
#include <artsim/math/dynmat.h>
#include <artsim/anim/pose_tree.h>

#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

namespace artsim {

struct Material;

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

    template <class Archive> void serialize(Archive& ar) {
        ar(type, limit_enabled, limit_min, limit_max, kp, kd, max_velocity);
    }

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
    glmx::ttransform<real> local_joint_pose;
    glmx::ttransform<real> local_link_pose;
    int parent_idx;
    Id<Material> mat_id;
    std::string obj_filename;

    template <class Archive> void serialize(Archive& ar) {
        ar(inertia, I_j, density, mass, col_shape, local_joint_pose, local_link_pose,
           parent_idx, mat_id, obj_filename);
    }

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

    template <class Archive> void serialize(Archive& ar) {
        ar(names, links, joints, initial_state, floating,
           joint_pos_dofs, joint_pos_dof_starts, joint_vel_dofs, joint_vel_dof_starts,
           num_pos_dofs, num_vel_dofs,
           parents, children_buffer, children_buffer_starts, bfs_iteration_order, build_finished);
    }

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
    artsim::ArticulatedBodySpec spec;
    Id<Material> mat_id;

    std::vector<real> q;
    std::vector<real> u;
    std::vector<real> udot;
    std::vector<real> tau;
    std::vector<glmx::tscrew<real>> f_ext;
    std::vector<glmx::tscrew<real>> f_c;
    std::vector<real> q_target;

    std::vector<glmx::ttransform<real>> global_link_trans;
    std::vector<glmx::ttransform<real>> global_joint_trans;
    std::vector<glmx::tscrew<real>> global_link_vel;

    std::vector<btCollisionObject*> bt_collision_objects;

    bool _is_static = false;
    bool _is_self_collision_enabled = false;

public:
    template <class Archive> void serialize(Archive& ar) {
        ar(spec, mat_id, q, u, udot, tau, f_ext, f_c, q_target,
           global_link_trans, global_joint_trans, global_link_vel, _is_static, _is_self_collision_enabled);
    }

    void init(artsim::ArticulatedBodySpec art_spec);
    void init(Id<ArticulatedBody> art_id, artsim::ArticulatedBodySpec art_spec, Id<Material> mat_id,
              btCollisionWorld* bt_collision_world,
              int col_filter_group_mask = btBroadphaseProxy::DefaultFilter,
              int col_filter_mask = btBroadphaseProxy::AllFilter,
              bool enable_self_collisions = false);

    void release(btCollisionWorld* bt_collision_world);

    void reset();
    void randomize_positions();

    const ArticulatedBodySpec& get_spec() const { return spec; }
    ArticulatedBodySpec& get_spec_mut() { return spec; }

    int get_num_pos_dofs() const { return spec.get_num_pos_dofs(); }
    int get_num_vel_dofs() const { return spec.get_num_vel_dofs(); }
    int get_num_joints() const { return spec.get_num_joints(); }
    int get_num_links() const { return spec.get_num_links(); }

    real* get_pos_buf() { return q.data(); }
    real* get_vel_buf() { return u.data(); }
    real* get_acc_buf() { return udot.data(); }
    real* get_internal_force_buf() { return tau.data(); }
    glmx::rscrew* get_external_force_buf() { return f_ext.data(); }
    glmx::rscrew* get_contact_force_buf() { return f_c.data(); }
    real* get_target_pos_buf() { return q_target.data(); }

    Id<Material> get_mat_id() { return mat_id; }
    void set_mat_id(Id<Material> new_mat_id) { mat_id = new_mat_id; }

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

    glmx::rtransform* get_global_joint_trans_buf() { return global_joint_trans.data(); }
    glmx::rtransform* get_global_link_trans_buf() { return global_link_trans.data(); }

    glmx::rtransform get_global_joint_trans(int joint_idx) const;
    glmx::rtransform get_global_link_trans(int link_idx) const;

    glmx::rscrew get_global_link_body_vel(int link_idx) const { return global_link_vel[link_idx]; }
    glm::rvec3 get_global_link_linvel(int link_idx) const {
        return global_link_trans[link_idx].R * global_link_vel[link_idx].v;
    }
    glm::rvec3 get_global_link_angvel(int link_idx) const {
        return global_link_trans[link_idx].R * global_link_vel[link_idx].w;
    }

    glm::rvec3 get_center_of_mass() const;

    bool is_self_collision_enabled() const { return _is_self_collision_enabled; }
    void enable_self_collisions() { _is_self_collision_enabled = true; }
    void disable_self_collisions() { _is_self_collision_enabled = false; }

    bool is_static() const { return _is_static; }
    void set_static() { _is_static = true; }
    void set_dynamic() { _is_static = false; }
};

}

#endif //ARTSIM_ART_BODY_H
