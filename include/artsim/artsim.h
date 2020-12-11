//
// Created by lasagnaphil on 20. 5. 19..
//

#ifndef ARTSIM_ARTSIM_H
#define ARTSIM_ARTSIM_H

#include "core/arena.h"

#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <artsim/math/se3.h>

#include <cstdint>
#include <cstdio>
#include <vector>
#include <unordered_map>

#define OUT
#define INOUT

namespace artsim {
    enum class JointType : uint8_t {
        Revolute, Prismatic, Spherical, Floating
    };

    struct Joint {
        JointType type;
        union {
            struct {
                glm::vec3 axis;
                bool enable_limit;
                float limit_min;
                float limit_max;
            } revolute;
            struct {
                glm::vec3 dir;
                bool enable_limit;
                float limit_min;
                float limit_max;
            } prismatic;
            struct {
            } spherical;
        };
        float damping = 0.0f;
        float max_velocity = 100.0f;

        static Joint revolute_free(glm::vec3 axis);
        static Joint revolute_limited(glm::vec3 axis, float limit_min, float limit_max);
        static Joint prismatic_free(glm::vec3 dir);
        static Joint prismatic_limited(glm::vec3 dir, float limit_min, float limit_max);
        static Joint spherical_free();
        static Joint floating();

        uint32_t pos_dof();
        uint32_t vel_dof();
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
                glm::vec3 size;
            } box;
            struct {
                float radius;
            } sphere;
        };

        static Shape make_ground();
        static Shape make_box(glm::vec3 size);
        static Shape make_sphere(float radius);

        float mass(float density);
        smat3x3 inertia(float density);
    };

    struct Material {
        float default_friction;
        float default_restitution;
    };

    struct MaterialPair {
        float friction;
        float restitution;
    };

    struct RigidBody {
        smat3x3 inertia;
        float mass;
        Shape shape;
        transform global_trans;
    };

    struct Link {
        smat3x3 inertia;
        float mass;
        Shape shape;
        transform local_joint_pose;
        transform local_link_pose;
        uint32_t parent_idx;
        Id<Material> mat_id;

        static Link create(const smat3x3& inertia, float mass, Shape shape,
                           transform local_joint_pose, transform local_link_pose,
                           int parent_idx, Id<Material> mat_id);
    };

    struct ArticulatedBody {
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

        ArticulatedBody(bool floating = false) : floating(floating) {}

        void add_link_and_joint(Link link, Joint joint) {
            if (joint.type == JointType::Floating) {
                if (!links.empty() || !joints.empty()) {
                    fprintf(stderr, "Error in ArticulatedBody::add_link_and_joint: "
                                    "Free joint can only be added at the root!\n");
                    return;
                }
                floating = true;
            }
            links.push_back(link);
            joints.push_back(joint);
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

        Id<Material> add_material(float default_friction = 1.0f,
                                  float default_restitution = 0.0f) {
            auto id = materials.make();
            auto ptr = materials.get(id);
            ptr->default_friction = default_friction;
            ptr->default_restitution = default_restitution;
            return id;
        }
        METHOD_GET_ID(Material, material, materials)
        METHOD_REMOVE_ID(Material, material, materials)

        void add_material_pair(Id<Material> mat1_id, Id<Material> mat2_id,
                               float friction, float restitution) {
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

    struct RigidBodyOrLink {
        bool is_link: 1;
        uint32_t index : 31;
        uint32_t generation;
        uint32_t link_idx;

        RigidBodyOrLink() = default;
        static RigidBodyOrLink from_articulation_link(Id<ArticulatedBody> id, uint32_t link_idx) {
            return RigidBodyOrLink { true, id.index, id.generation, link_idx };
        }
        static RigidBodyOrLink from_rigid_body(Id<RigidBody> id) {
            return RigidBodyOrLink { false, id.index, id.generation, 0 };
        }
        std::pair<Id<ArticulatedBody>, uint32_t> get_articulation_link() const {
            return {Id<ArticulatedBody>{index, generation}, link_idx};
        }
        Id<RigidBody> get_rigid_body_id() const { return Id<RigidBody>{index, generation}; }
    };

    struct ContactPoint {
        transform T_global;
        float depth;
        RigidBodyOrLink body1_id;
        RigidBodyOrLink body2_id;

        ContactPoint() = default;
        ContactPoint(glm::vec3 pos, glm::vec3 normal, glm::vec3 tangent,
                     float depth, RigidBodyOrLink body1_id, RigidBodyOrLink body2_id)
              : T_global(pos, glm::mat3(tangent, glm::cross(normal, tangent), normal)),
                depth(depth), body1_id(body1_id), body2_id(body2_id) {}
    };

    struct Frame {
        RigidBodyOrLink body;
        artsim::transform T_local;

        static Frame from_articulation(Id<ArticulatedBody> id, uint32_t link_idx,
                                       const artsim::transform& T_local) {
            return {RigidBodyOrLink::from_articulation_link(id, link_idx), T_local};
        }
    };
}

#endif //ARTSIM_ARTSIM_H
