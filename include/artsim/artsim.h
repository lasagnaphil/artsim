//
// Created by lasagnaphil on 20. 5. 19..
//

#ifndef ARTSIM_ARTSIM_H
#define ARTSIM_ARTSIM_H

#include "core/arena.h"

#include <cstdint>
#include <vector>
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <artsim/math/se3.h>

namespace artsim {
    enum class JointType : uint8_t {
        Free, Revolute, Prismatic, Spherical, Euler
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
                glm::vec3 limit_cone_dir;
                bool enable_limit;
                float limit_cone_angle;
            } expmap;
            struct {
                glm::vec3 limit_min;
                bool enable_limit;
                glm::vec3 limit_max;
            } euler;
            struct {
                float linvel_damping;
                float angvel_damping;
                float max_linvel;
                float max_angvel;
            } free;
        };
        float damping = 0.0f;
        float max_velocity = 100.0f;
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

        float mass(float density);
        glm::mat3 inertia(float density);
    };

    struct Material {
        float default_friction;
        float default_restitution;
        float default_restitution_threshold;
    };

    struct Link {
        glm::mat3 inertia; // G_i
        float mass;
        Shape shape;
        artsim::transform global_pose; // M_i
        artsim::transform local_joint_pose;
        Id<Link> parent_id;
        Id<Material> mat_id;
    };

    struct MaterialPair {
        float friction;
        float restitution;
        float restitution_threshold;
    };

    struct ArticulatedBody {
        std::vector<Id<Joint>> joints;
        std::vector<Id<Link>> links;
    };

    struct World {
        Arena<Joint> joints;
        Arena<Link> links;
        Arena<ArticulatedBody> articulatedBodies;
        Arena<Material> materials;
        Arena<MaterialPair> materialPairs;

#define METHOD_GET_ID(TYPE, MEMBER) TYPE* get##TYPE(Id<TYPE> id) { return MEMBER.get(id); }

        METHOD_GET_ID(Joint, joints)
        METHOD_GET_ID(Link, links)
        METHOD_GET_ID(ArticulatedBody, articulatedBodies)
        METHOD_GET_ID(MaterialPair, materialPairs)

#undef METHOD_GET_ID

        World() {}

        Id<Joint> add_floating_joint();
        Id<Joint> add_revolute_joint(glm::vec3 axis);
        Id<Joint> add_revolute_joint(glm::vec3 axis, float limit_min, float limit_max);
        Id<Joint> add_prismatic_joint(glm::vec3 dir);
        Id<Joint> add_prismatic_joint(glm::vec3 dir, float limit_min, float limit_max);
        Id<Joint> add_spherical_joint();

        Id<Shape> add_box_shape(glm::vec3 size);
        Id<Shape> add_sphere_shape(float radius);

        Id<Link> add_link(glm::mat3 inertia, float mass, Shape shape,
                          artsim::transform global_link_pose, artsim::transform global_joint_pose,
                          int parent_idx, uint32_t mat_idx);
        Id<Link> add_floating_link(glm::mat3 inertia, float mass, Shape shape,
                                   artsim::transform global_link_pose, uint32_t mat_idx);
    };

    struct MaterialDatabase {
        std::vector<std::vector<MaterialPair>> material_pairs; // TOOD: make this more data-oriented

        MaterialPair get_material_pair(uint32_t mat1_idx, uint32_t mat2_idx) {
            return material_pairs[mat1_idx][mat2_idx];
        }
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
