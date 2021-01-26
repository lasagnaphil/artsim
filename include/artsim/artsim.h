//
// Created by lasagnaphil on 20. 5. 19..
//

#ifndef ARTSIM_ARTSIM_H
#define ARTSIM_ARTSIM_H

#include "core/arena.h"

#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <artsim/math/se3.h>

#include <cstdint>
#include <cstdio>
#include <vector>
#include <unordered_map>
#include <string>

#define OUT
#define INOUT

namespace artsim {

#ifdef ARTSIM_USE_DOUBLE
using real = double;
#else
using real = float;
#endif

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

inline btTransform btconv(const ttransform<real>& T) {
    return btTransform(btconv(T.R), btconv(T.v));
}

inline glm::tvec3<real> glmconv(const btVector3& v) {
    return {v.x(), v.y(), v.z()};
}

inline glm::tmat3x3<real> glmconv(const btMatrix3x3& M) {
    return {glmconv(M.getColumn(0)), glmconv(M.getColumn(1)), glmconv(M.getColumn(2))};
}

inline ttransform<real> glmconv(const btTransform& T) {
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

        uint32_t pos_dof() {
            switch (type) {
                JOINT_DOF_1_CASE { return 1; }
                case JOINT_TYPE_SPHERICAL: return 4;
                case JOINT_TYPE_FLOATING: return 7;
                default: return 0;
            }
        }
        uint32_t vel_dof() {
            switch (type) {
                JOINT_DOF_1_CASE { return 1; }
                case JOINT_TYPE_SPHERICAL: return 3;
                case JOINT_TYPE_FLOATING: return 6;
                default: return 0;
            }
        }
    };

    struct Shape {
        enum class Type {
            Ground, Box, Sphere
        };
        Type type;

        union {
            struct {
            } ground;
            struct {
                glm::tvec3<real> size;
            } box;
            struct {
                real radius;
            } sphere;
        };

        static Shape make_ground();
        static Shape make_box(glm::vec3 size);
        static Shape make_sphere(real radius);

        real mass(real density);
        tsmat3x3<real> inertia(real density);
    };

    struct Material {
        real default_friction;
        real default_restitution;
    };

    struct MaterialPair {
        real friction;
        real restitution;
    };

    struct RigidBody {
        tsmat3x3<real> inertia;
        real mass;
        Shape shape;
        ttransform<real> global_trans;
    };

    struct Link {
        tsmat3x3<real> inertia; // inertia from link frame
        tspmat<real> I_j; // Spatial mass matrix from joint frame
        real mass;
        Shape shape;
        btCollisionShape* bt_shape;
        ttransform<real> local_joint_pose;
        ttransform<real> local_link_pose;
        uint32_t parent_idx;
        Id<Material> mat_id;
        btCollisionObject* bt_collision_object;

        static Link create(const tsmat3x3<real>& inertia, real mass, Shape shape,
                           ttransform<real> local_joint_pose, ttransform<real> local_link_pose,
                           int parent_idx, Id<Material> mat_id);
    };

    struct ArticulatedBody {
        std::vector<std::string> names;
        std::vector<Link> links;
        std::vector<Joint> joints;

        bool floating = false;

        std::vector<uint32_t> joint_pos_dofs;
        std::vector<uint32_t> joint_pos_dof_starts;
        std::vector<uint32_t> joint_vel_dofs;
        std::vector<uint32_t> joint_vel_dof_starts;
        uint32_t num_pos_dofs = 0;
        uint32_t num_vel_dofs = 0;

        std::vector<int> parents;
        std::vector<uint32_t> children_buffer;
        std::vector<uint32_t> children_buffer_starts;
        std::vector<uint32_t> bfs_iteration_order;

        bool build_finished = false;

        btCollisionWorld* bt_collision_world = nullptr;

        ArticulatedBody(bool floating = false) : floating(floating) {}

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

        void setup();

        uint32_t get_num_joints() const {
            return joints.size();
        }

        uint32_t get_num_pos_dofs() const {
            return num_pos_dofs;
        };

        uint32_t get_num_vel_dofs() const {
            return num_vel_dofs;
        };

        uint32_t get_num_children(uint32_t joint_idx) const {
            return children_buffer_starts[joint_idx+1] - children_buffer_starts[joint_idx];
        }

        const uint32_t* get_children(uint32_t joint_idx) const {
            return &children_buffer[children_buffer_starts[joint_idx]];
        }
    };

    struct pair_hash {
        template <class T1, class T2>
        std::size_t operator () (std::pair<T1, T2> const &v) const
        {
            using std::hash;
            return hash<T1>()(v.first) ^ (hash<T2>()(v.second) << 1);
        }
    };

#define METHOD_GET_ID(TYPE, NAME, MEMBER) TYPE* get_##NAME(Id<TYPE> id) { return MEMBER.get(id); }
#define METHOD_REMOVE_ID(TYPE, NAME, MEMBER) void remove_##NAME(Id<TYPE> id) { MEMBER.release(id); }

    struct MaterialDB {
        Arena<Material> materials;
        std::unordered_map<std::pair<Id<Material>, Id<Material>>, MaterialPair, pair_hash> material_pairs;

        Id<Material> add_material(real default_friction = 1.0f,
                                  real default_restitution = 0.0f) {
            auto id = materials.make();
            auto ptr = materials.get(id);
            ptr->default_friction = default_friction;
            ptr->default_restitution = default_restitution;
            return id;
        }
        METHOD_GET_ID(Material, material, materials)
        METHOD_REMOVE_ID(Material, material, materials)

        void add_material_pair(Id<Material> mat1_id, Id<Material> mat2_id,
                               real friction, real restitution) {
            material_pairs[std::make_pair(mat1_id, mat2_id)] = MaterialPair {friction, restitution};
        }

        void remove_material_pair(Id<Material> mat1_id, Id<Material> mat2_id) {
            material_pairs.erase(std::make_pair(mat1_id, mat2_id));
        }

    };

    struct World {
        Arena<RigidBody> rigid_bodies;
        Arena<ArticulatedBody> articulated_bodies;

        World() {}

        Id<ArticulatedBody> add_articulated_body(bool floating = false) {
            auto id = articulated_bodies.make();
            auto ptr = articulated_bodies.get(id);
            ptr->floating = floating;
            return id;
        }

        METHOD_GET_ID(ArticulatedBody, articulated_body, articulated_bodies)
        METHOD_REMOVE_ID(ArticulatedBody, articulated_body, articulated_bodies)

        void add_link_and_joint_to_articulation(Id<ArticulatedBody> art_id, Link link, Joint joint) {
            auto art = articulated_bodies.get(art_id);
            art->links.push_back(link);
            art->joints.push_back(joint);
        }

        void build_articulation(Id<ArticulatedBody> art_id) {
            auto art = articulated_bodies.get(art_id);
            art->setup();
        }

#undef METHOD_GET_ID
#undef METHOD_DELETE_ID

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
            BodyId body_id;
            body_id.index = 0x80000000 | ((id.index & 0x0000ffff) << 16) | link_idx;
            body_id.generation = id.generation;
            return body_id;
        }
        static BodyId from_rigid_body(Id<RigidBody> id) {
            BodyId body_id;
            body_id.index = id.index;
            body_id.generation = id.generation;
            return body_id;
        }
        static BodyId from_ground() {
            return {0, 0};
        }
        std::pair<Id<ArticulatedBody>, uint32_t> get_articulation_id() const {
            if (!is_articulation()) return {Id<ArticulatedBody>::null(), 0};
            uint32_t art_index = (index & 0x7fff0000) >> 16;
            uint32_t art_body_index = index & 0x0000ffff;
            return {Id<ArticulatedBody>{art_index, generation}, art_body_index};
        }
        Id<RigidBody> get_rigid_body_id() const {
            if (is_articulation()) return Id<RigidBody>::null();
            return Id<RigidBody>{index, generation};
        }
        bool is_articulation() const {
            return (index & 0x80000000) != 0;
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
        glm::tvec3<real> pos;
        glm::tvec3<real> normal;
        real depth;
        real area;
        BodyId body1_id;
        BodyId body2_id;
    };

    struct Frame {
        BodyId body;
        artsim::transform T_local;

        static Frame from_articulation(Id<ArticulatedBody> id, uint32_t link_idx,
                                       const artsim::transform& T_local) {
            return {BodyId::from_articulation_link(id, link_idx), T_local};
        }
    };
}

#endif //ARTSIM_ARTSIM_H
