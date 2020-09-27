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
#include <vector>
#include <unordered_map>

#define OUT

namespace artsim {
    enum class JointType : uint8_t {
        Revolute, Prismatic, Spherical
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

        uint32_t joint_dof();
    };

    struct Shape {
        enum class Type {
            Box, Sphere
        };
        Type type;

        union {
            struct {
                glm::vec3 size;
            } box;
            struct {
                float radius;
            } sphere;
        };

        static Shape make_box(glm::vec3 size);
        static Shape make_sphere(float radius);

        float mass(float density);
        glm::mat3 inertia(float density);
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
        glm::mat3 inertia;
        float mass;
        Shape shape;
        transform global_trans;
    };

    struct Link {
        glm::mat3 inertia;
        float mass;
        Shape shape;
        transform local_link_pose;
        transform local_joint_pose;
        // transform global_link_pose;
        // transform global_joint_pose;
        uint32_t parent_idx;
        Id<Material> mat_id;

        static Link create(glm::mat3 inertia, float mass, Shape shape,
                           transform local_link_pose, transform local_joint_pose,
                           int parent_idx, Id<Material> mat_id);
    };

    struct ArticulatedBody {
        std::vector<Link> links;
        std::vector<Joint> joints;
        transform root_transform = transform();

        std::vector<uint32_t> joint_dofs;
        std::vector<uint32_t> joint_dof_starts;

        std::vector<int> parents;
        std::vector<uint32_t> children_buffer;
        std::vector<uint32_t> children_buffer_starts;
        std::vector<uint32_t> bfs_iteration_order;

        uint32_t num_dofs = 0;
        bool floating;

        bool build_finished = false;

        uint32_t get_num_joints() const {
            return joints.size();
        }

        uint32_t get_num_dofs() const {
            return num_dofs;
        };

        uint32_t get_num_children(uint32_t joint_idx) const {
            return children_buffer_starts[joint_idx+1] - children_buffer_starts[joint_idx];
        }

        const uint32_t* get_children(uint32_t joint_idx) const {
            return &children_buffer[children_buffer_starts[joint_idx]];
        }

        ArticulatedBody(bool floating = false) : floating(floating) {}

        /*
        void add_joint_link_local(Joint joint, RigidBody rigid_body,
                                  transform local_link_pose, transform local_joint_pose,
                                  int parent_idx, Id<Material> mat_id);

        void add_joint_link_global(Joint joint, RigidBody rigid_body,
                                   transform global_link_pose, transform global_joint_pose,
                                   int parent_idx, Id<Material> mat_id);
                                   */

        void add_link_and_joint(Link link, Joint joint) {
            links.push_back(link);
            joints.push_back(joint);
        }

        void setup();
    };

    struct pair_hash {
        template <class T1, class T2>
        std::size_t operator () (std::pair<T1, T2> const &v) const
        {
            using std::hash;
            return hash<T1>()(v.first) ^ (hash<T2>()(v.second) << 1);
        }
    };

    struct World {
        Arena<RigidBody> rigid_bodies;
        Arena<ArticulatedBody> articulated_bodies;

        Arena<Material> materials;
        std::unordered_map<std::pair<Id<Material>, Id<Material>>, MaterialPair, pair_hash> material_pairs;

        World() {}

#define METHOD_GET_ID(TYPE, NAME, MEMBER) TYPE* get_##NAME(Id<TYPE> id) { return MEMBER.get(id); }
#define METHOD_REMOVE_ID(TYPE, NAME, MEMBER) void remove_##NAME(Id<TYPE> id) { MEMBER.release(id); }

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

    struct ContactPoint {
        artsim::transform trans; // z-axis of the contact frame is the normal
        float depth;
        Id<Link> link1_id;
        Id<Link> link2_id;

        ContactPoint() = default;
        ContactPoint(glm::vec3 pos, glm::vec3 normal, glm::vec3 tangent,
                float depth, Id<Link> link1_id, Id<Link> link2_id)
              : trans(pos, glm::quat_cast(glm::mat3(tangent, glm::cross(normal, tangent), normal))),
                depth(depth), link1_id(link1_id), link2_id(link2_id) {}
    };
}

#endif //ARTSIM_ARTSIM_H
