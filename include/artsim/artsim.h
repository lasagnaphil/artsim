//
// Created by lasagnaphil on 20. 5. 19..
//

#ifndef ARTSIM_ARTSIM_H
#define ARTSIM_ARTSIM_H

#include "core/arena.h"
#include "types.h"

#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <artsim/math/se3.h>
#include <artsim/math/dynmat.h>
#include <artsim/obj_file.h>

#include <cstdint>
#include <cstdio>
#include <vector>
#include <unordered_map>
#include <string>
#include <tiny_obj_loader.h>

namespace artsim {

#define SHOW_LOG

#ifdef SHOW_LOG
#define output_log(...) printf(__VA_ARGS__)
#else
#define output_log(...)
#endif


inline btVector3 btconv(const glm::tvec3<real>& v) {
    return btVector3(v.x, v.y, v.z);
}

inline btMatrix3x3 btconv(const glm::tmat3x3<real>& M) {
    return btMatrix3x3(M[0][0], M[1][0], M[2][0], M[0][1], M[1][1], M[2][1], M[0][2], M[1][2], M[2][2]);
}

inline btQuaternion btconv(const glm::tquat<real>& q) {
    return btQuaternion(q.x, q.y, q.z, q.w);
}

inline btTransform btconv(const glmx::ttransform<real>& T) {
    return btTransform(btconv(T.R), btconv(T.v));
}

inline btTransform btconv(const glmx::tquat_transform<real>& T) {
    return btTransform(btconv(T.q), btconv(T.v));
}

inline glm::tvec3<real> glmconv(const btVector3& v) {
    return {v.x(), v.y(), v.z()};
}

inline glm::tmat3x3<real> glmconv(const btMatrix3x3& M) {
    return {glmconv(M.getColumn(0)), glmconv(M.getColumn(1)), glmconv(M.getColumn(2))};
}

inline glmx::ttransform<real> glmconv(const btTransform& T) {
    return {glmconv(T.getOrigin()), glmconv(T.getBasis())};
}

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

#define JOINT_DOF_1_CASE case JOINT_TYPE_REVOLUTE_X: case JOINT_TYPE_REVOLUTE_Y: case JOINT_TYPE_REVOLUTE_Z: \
                         case JOINT_TYPE_PRISMATIC_X: case JOINT_TYPE_PRISMATIC_Y: case JOINT_TYPE_PRISMATIC_Z:

struct Joint {
    constexpr static real default_kp = 10.0;
    constexpr static real default_kd = 0.1;
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

struct CollisionShape {
    enum class Type {
        Ground, Box, Sphere, Mesh
    };
    Type type;
    glm::tvec3<real> scale;

    union {
        struct {
        } ground;
        struct {
        } box;
        struct {
        } sphere;
        struct {
            tinyobj::attrib_t* attrib;
            tinyobj::shape_t* shapes;
            int num_shapes;
        } mesh;
    };
    btCollisionShape* bt_shape;

    static CollisionShape make_ground();
    static CollisionShape make_box(glm::vec3 size);
    static CollisionShape make_sphere(real radius);
    static CollisionShape make_mesh(const tinyobj::attrib_t* attrib, const tinyobj::shape_t* shapes, int num_shapes, glm::rvec3 scale = glm::rvec3(1));

    real mass(real density);
    glmx::tsmat3x3<real> inertia(real density);
};

struct RenderShape {
    enum class Type {
        Box, Sphere, Mesh
    };
    Type type;
    glm::vec3 scale = glm::vec3(1);
    glm::vec3 color = glm::vec3(1, 0, 0);

    union {
        struct {
        } box;
        struct {
        } sphere;
        struct {
            tinyobj::attrib_t* attrib;
            tinyobj::shape_t* shapes;
            int num_shapes;
        } mesh;
    };

    static RenderShape make_from_collision_shape(const CollisionShape& col);
    static RenderShape make_box(glm::vec3 size);
    static RenderShape make_sphere(float radius);
    static RenderShape make_mesh(const tinyobj::attrib_t* attrib, const tinyobj::shape_t* shapes, int num_shapes, glm::vec3 scale = glm::vec3(1));
};

struct Material {
    real friction = 1.0f;
    real restitution = 0.0f;
    real restitution_threshold = 0.01f;
};

struct RigidBodySpec {
    glmx::tsmat3x3<real> inertia;
    real mass;
    CollisionShape col_shape;
    RenderShape render_shape;
    glmx::ttransform<real> global_trans;
    bool is_static = false;
};

struct RigidBody {
    RigidBodySpec spec;
    glm::rvec3 pos;
    glm::rvec3 vel;
    glm::rquat rot;
    glm::rvec3 angvel;
    Id<Material> mat_id;
    btCollisionObject* bt_collision_object;

    void init(RigidBodySpec rb_spec);

    void init(Id<RigidBody> rb_id, RigidBodySpec rb_spec, Id<Material> mat_id,
              btCollisionWorld* bt_collision_world);

    void release(btCollisionWorld* bt_world);
};

struct Link {
    glmx::tsmat3x3<real> inertia; // inertia from link frame
    glmx::tspmat<real> I_j; // Spatial mass matrix from joint frame
    real mass;
    CollisionShape col_shape;
    RenderShape render_shape;
    glmx::ttransform<real> local_joint_pose;
    glmx::ttransform<real> local_link_pose;
    int parent_idx;
    Id<Material> mat_id;

    static Link create(const glmx::tsmat3x3<real>& inertia, real mass,
                       CollisionShape col_shape,
                       glmx::ttransform<real> local_joint_pose, glmx::ttransform<real> local_link_pose,
                       int parent_idx, Id<Material> mat_id);

    static Link create(const glmx::tsmat3x3<real>& inertia, real mass,
                       CollisionShape col_shape, RenderShape render_shape,
                       glmx::ttransform<real> local_joint_pose, glmx::ttransform<real> local_link_pose,
                       int parent_idx, Id<Material> mat_id);

};

struct ArticulatedBodySpec {
    std::vector<std::string> names;
    std::vector<Link> links;
    std::vector<Joint> joints;

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

    void add_link_and_joint(Link link, Joint joint, const std::string& name = "") {
        if (joint.type == JOINT_TYPE_FLOATING) {
            if (!links.empty() || !joints.empty()) {
                fprintf(stderr, "Error in ArticulatedBody::add_link_and_joint: "
                                "Free joint can only be added at the root!\n");
                return;
            }
            floating = true;
        }
        links.push_back(link);
        joints.push_back(joint);
        names.push_back(name);
    }

    void build();

    int get_num_joints() const {
        return joints.size();
    }

    int get_num_links() const {
        return links.size();
    }

    int get_num_pos_dofs() const {
        return num_pos_dofs;
    };

    int get_num_vel_dofs() const {
        return num_vel_dofs;
    };

    int get_num_children(int joint_idx) const {
        return children_buffer_starts[joint_idx+1] - children_buffer_starts[joint_idx];
    }

    const int * get_children(int joint_idx) const {
        return &children_buffer[children_buffer_starts[joint_idx]];
    }

    int get_index(const char* name) const {
        int i;
        for (i = 0; i < names.size(); i++) {
            if (names[i] == name) break;
        }
        if (i == names.size()) return -1;
        else return i;
    }

    void scale_link(int link_idx, const glm::rvec3& scale, bool scale_shapes) {
        auto& link = links[link_idx];
        if (scale_shapes) {
            link.render_shape.scale *= scale;
            link.col_shape.scale *= scale;
        }
        link.local_link_pose.v = glm::rvec3(scale) * link.local_link_pose.v;
        uint32_t num_children = get_num_children(link_idx);
        const int* children = get_children(link_idx);
        for (int i = 0; i < num_children; i++) {
            uint32_t child_idx = children[i];
            links[child_idx].local_joint_pose.v =
                    glm::rvec3(scale) * links[child_idx].local_joint_pose.v;
        }
    }

    void scale_link(int link_idx, const glm::rmat3& rot, const glm::rvec3& scale) {
        auto& link = links[link_idx];
        auto T = rot * glm::rmat3(scale[0], 0, 0, 0, scale[1], 0, 0, 0, scale[2]) * glm::transpose(rot);
        link.local_link_pose.v = T * link.local_link_pose.v;
        uint32_t num_children = get_num_children(link_idx);
        const int* children = get_children(link_idx);
        for (int i = 0; i < num_children; i++) {
            uint32_t child_idx = children[i];
            links[child_idx].local_joint_pose.v =
                    T * links[child_idx].local_joint_pose.v;
        }
    }
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
    std::vector<glmx::ttransform<real>> T_link_globals;
    std::vector<glmx::ttransform<real>> T_joint_globals;

    std::vector<btCollisionObject*> bt_collision_objects;

public:
    void init(artsim::ArticulatedBodySpec art_spec);
    void init(Id<ArticulatedBody> art_id, artsim::ArticulatedBodySpec art_spec, Id<Material> mat_id,
              btCollisionWorld* bt_collision_world);
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
    void integrate(real dt);
    void simulate(const glm::rvec3& gravity, real dt);

    void mass_matrix(OUT glmx::dynmat_view<real> M, real dt = 0);
    void multiply_inverse_mass_matrix(glmx::dynmat_view<real> X, OUT glmx::dynmat_view<real> Minv_X, real dt = 0);

    glmx::rtransform* get_global_joint_trans_buf() { return T_joint_globals.data(); }
    glmx::rtransform* get_global_link_trans_buf() { return T_link_globals.data(); }

    glmx::rtransform get_global_joint_trans(int joint_idx) const;
    glmx::rtransform get_global_link_trans(int link_idx) const;
};

struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (std::pair<T1, T2> const &v) const
    {
        using std::hash;
        return hash<T1>()(v.first) ^ (hash<T2>()(v.second) << 1);
    }
};

struct BodyId {
    /*
     * Memory layout:
     *
    bool is_art: 1;
    union {
        uint32_t rigid_body_idx: 31;
        struct {
            uint16_t art_idx: 15;
            uint16_t art_body_idx: 16;
        };
    };
     */
    uint32_t index;
    uint32_t generation;

    friend bool operator==(BodyId id1, BodyId id2);
    friend bool operator!=(BodyId id1, BodyId id2);

    static BodyId from_articulation_link(Id<ArticulatedBody> id, uint16_t link_idx) {
        auto [index, generation] = id.to_int32s();
        BodyId body_id;
        body_id.index = 0x80000000 | ((index & 0x0000ffff) << 16) | link_idx;
        body_id.generation = generation;
        return body_id;
    }
    static BodyId from_rigid_body(Id<RigidBody> id) {
        auto [index, generation] = id.to_int32s();
        BodyId body_id;
        body_id.index = index;
        body_id.generation = generation;
        return body_id;
    }
    static BodyId from_ground() {
        return {0, 0};
    }
    std::pair<Id<ArticulatedBody>, uint32_t> get_articulation_id() const {
        if (!is_articulation()) return {Id<ArticulatedBody>::null(), 0};
        uint32_t art_index = (index & 0x7fff0000) >> 16;
        uint32_t art_body_index = index & 0x0000ffff;
        return {Id<ArticulatedBody>::from_int32s(art_index, generation), art_body_index};
    }
    Id<RigidBody> get_rigid_body_id() const {
        if (is_articulation()) return Id<RigidBody>::null();
        return Id<RigidBody>::from_int32s(index, generation);
    }

    bool is_rigid_body() const {
        return (index & 0x80000000) == 0;
    }
    bool is_articulation() const {
        return (index & 0x80000000) != 0;
    }
    bool is_ground() const {
        return index == 0 && generation == 0;
    }
};

inline bool operator==(BodyId id1, BodyId id2) {
    return id1.index == id2.index && id1.generation == id2.generation;
}
inline bool operator!=(BodyId id1, BodyId id2) {
    return id1.index != id2.index || id1.generation != id2.generation;
}

struct ContactPoint {
    btPersistentManifold* bt_manifold;
    btManifoldPoint* bt_manifold_point;
    glm::tvec3<real> pos;
    glm::tvec3<real> normal;
    real depth;
    real area;
    BodyId body1_id;
    BodyId body2_id;
};

struct Frame {
    BodyId body;
    glmx::transform T_local;

    static Frame from_articulation(Id<ArticulatedBody> id, int link_idx,
                                   const glmx::transform& T_local) {
        return {BodyId::from_articulation_link(id, link_idx), T_local};
    }
};

#define METHOD_GET_ID(TYPE, NAME, MEMBER) TYPE* get_##NAME(Id<TYPE> id) { return MEMBER.get(id); }
#define METHOD_REMOVE_ID(TYPE, NAME, MEMBER) void remove_##NAME(Id<TYPE> id) { MEMBER.release(id); }

struct MaterialDB {
    Arena<Material> materials;
    std::unordered_map<std::pair<Id<Material>, Id<Material>>, Material, pair_hash> material_pairs;

    Id<Material> add_material(real default_friction = 1.0f,
                              real default_restitution = 0.0f,
                              real default_restitution_threshold = 0.01f) {
        auto id = materials.make();
        auto ptr = materials.get(id);
        ptr->friction = default_friction;
        ptr->restitution = default_restitution;
        ptr->restitution_threshold = default_restitution_threshold;
        return id;
    }
    METHOD_GET_ID(Material, material, materials)
    METHOD_REMOVE_ID(Material, material, materials)

    void set_material_pair(Id<Material> mat1_id, Id<Material> mat2_id,
                           real friction, real restitution, real restitution_threshold) {
        material_pairs[std::make_pair(mat1_id, mat2_id)] = Material{friction, restitution, restitution_threshold};
    }

    Material get_material_pair(Id<Material> mat1_id, Id<Material> mat2_id) {
        auto it = material_pairs.find({mat1_id, mat2_id});
        if (it == material_pairs.end()) {
            Material* mat1 = materials.get(mat1_id);
            Material* mat2 = materials.get(mat2_id);
            Material mat;
            mat.friction = glm::max(mat1->friction, mat2->friction);
            mat.restitution = glm::min(mat1->restitution, mat2->restitution);
            mat.restitution_threshold = glm::max(mat1->restitution_threshold, mat2->restitution_threshold);
            return mat;
        }
        else {
            return it->second;
        }
    }
};

enum class ContactSolverType {
    PGS, Bisection, NCP
};

struct WorldConfig {
    glm::rvec3 gravity = {0.0, -9.8, 0.0};
    real dt = 1.0 / 240.0;
    ContactSolverType contact_solver_type = ContactSolverType::PGS;
    int max_iters = 4;
};

class World {
private:
    Arena<RigidBody> rigid_bodies;
    Arena<ArticulatedBody> articulated_bodies;
    MaterialDB material_db;

    Arena<ContactPoint> contact_points;
    btCollisionWorld* bt_collision_world = nullptr;
    btCollisionObject* bt_plane_col = nullptr;

    WorldConfig cfg;

public:
    void init(WorldConfig world_cfg);

    glm::rvec3 get_gravity() const { return cfg.gravity; }
    void set_gravity(const glm::rvec3& gravity) { cfg.gravity = gravity; }

    real get_timestep() const { return cfg.dt; }
    void set_timestep(real dt) { cfg.dt = dt; }

    Id<RigidBody> add_rigid_body(const RigidBodySpec& spec, Id<Material> mat_id) {
        auto id = rigid_bodies.make();
        auto ptr = rigid_bodies.get(id);
        ptr->init(id, spec, mat_id, bt_collision_world);
        return id;
    }

    RigidBody* get_rigid_body(Id<RigidBody> id) {
        return rigid_bodies.get(id);
    }

    bool remove_rigid_body(Id<RigidBody> id) {
        auto ptr = rigid_bodies.try_get(id);
        if (!ptr) return false;
        ptr->release(bt_collision_world);
        rigid_bodies.release(id);
        return true;
    }

    Id<ArticulatedBody> add_articulated_body(const ArticulatedBodySpec& spec, Id<Material> mat_id) {
        auto id = articulated_bodies.make();
        auto ptr = articulated_bodies.get(id);
        ptr->init(id, spec, mat_id, bt_collision_world);
        return id;
    }

    ArticulatedBody* get_articulated_body(Id<ArticulatedBody> id) {
        return articulated_bodies.get(id);
    }

    bool remove_articulated_body(Id<ArticulatedBody> id) {
        auto ptr = articulated_bodies.try_get(id);
        if (!ptr) return false;
        ptr->release(bt_collision_world);
        articulated_bodies.release(id);
        return true;
    }

    Id<Material> add_material(real default_friction = 1.0f,
                              real default_restitution = 0.0f,
                              real default_restitution_threshold = 0.01f) {
        return material_db.add_material(default_friction, default_restitution, default_restitution_threshold);
    }

    Material* get_material(Id<Material> id) {
        return material_db.get_material(id);
    }

    void remove_material(Id<Material> id) {
        return material_db.remove_material(id);
    }

    void set_material_pair(Id<Material> mat1_id, Id<Material> mat2_id,
                           real friction, real restitution, real restitution_threshold) {
        material_db.set_material_pair(mat1_id, mat2_id, friction, restitution, restitution_threshold);
    }

    Id<RigidBody> add_plane(Id<Material> mat_id) {
        RigidBodySpec spec;
        const real inf = std::numeric_limits<real>::infinity();
        spec.mass = inf;
        spec.inertia = glmx::rsmat3x3(inf);
        spec.col_shape = CollisionShape::make_ground();
        // spec.render_shape = RenderShape::make_from_collision_shape(spec.col_shape);
        spec.global_trans = glmx::rtransform(glmx::IDENTITY);
        spec.is_static = true;
        return add_rigid_body(spec, mat_id);
    }

    void simulate(real dt);

private:
    void integrate_with_contacts();
};

#undef METHOD_GET_ID
#undef METHOD_DELETE_ID

}

#endif //ARTSIM_ARTSIM_H
