//
// Created by lasagnaphil on 20. 5. 19..
//

#include "artsim/artsim.h"
#include "artsim/math/se3.h"

using namespace artsim;

Joint Joint::revolute_free(glm::vec3 axis) {
    Joint joint;
    joint.type = JointType::Revolute;
    joint.revolute.axis = axis;
    joint.revolute.enable_limit = false;
    return joint;
}

Joint Joint::revolute_limited(glm::vec3 axis, float limit_min, float limit_max) {
    Joint joint;
    joint.type = JointType::Revolute;
    joint.revolute.axis = axis;
    joint.revolute.enable_limit = true;
    joint.revolute.limit_min = limit_min;
    joint.revolute.limit_max = limit_max;
    return joint;
}

Joint Joint::prismatic_free(glm::vec3 dir) {
    Joint joint;
    joint.type = JointType::Prismatic;
    joint.prismatic.dir = dir;
    joint.prismatic.enable_limit = false;
    return joint;
}

Joint Joint::prismatic_limited(glm::vec3 dir, float limit_min, float limit_max) {
    Joint joint;
    joint.type = JointType::Prismatic;
    joint.prismatic.dir = dir;
    joint.prismatic.enable_limit = false;
    joint.prismatic.limit_min = limit_min;
    joint.prismatic.limit_max = limit_max;
    return joint;
}

Joint Joint::spherical_free() {
    Joint joint;
    joint.type = JointType::Spherical;
    return joint;
}

uint32_t Joint::joint_dof() {
    switch (type) {
        case JointType::Revolute: return 1; break;
        case JointType::Prismatic: return 1; break;
        case JointType::Spherical: return 3; break;
    }
}

float Shape::mass(float density) {
    switch (type) {
        case Type::Box: {
            return density * box.size.x * box.size.y * box.size.z;
        } break;
        case Type::Sphere: {
            return 4.f / 3.f * glm::pi<float>() * sphere.radius * sphere.radius * sphere.radius;
        } break;
    }
}

glm::mat3 Shape::inertia(float density) {
    glm::vec3 I;
    switch (type) {
        case Type::Box: {
            const glm::vec3& s = box.size;
            I = mass(density) * glm::vec3(s.y*s.y + s.z*s.z, s.z*s.z + s.x*s.x, s.x*s.x + s.y*s.y) / 12.f;
        } break;
        case Type::Sphere: {
            float r = sphere.radius;
            I = 0.4f * mass(density) * glm::vec3(r*r);
        } break;
    }
    return glm::mat3(I.x, 0, 0, 0, I.y, 0, I.z, 0, 0);
}

Shape Shape::make_box(glm::vec3 size) {
    Shape shape;
    shape.box.size = size;
    return shape;
}

Shape Shape::make_sphere(float radius) {
    Shape shape;
    shape.sphere.radius = radius;
    return shape;
}

Link Link::create(glm::mat3 inertia, float mass, Shape shape, transform local_link_pose, transform local_joint_pose,
                  int parent_idx, Id<Material> mat_id) {
    Link link;
    link.inertia = inertia;
    link.mass = mass;
    link.shape = shape;
    link.local_link_pose = local_link_pose;
    link.local_joint_pose = local_joint_pose;
    link.parent_idx = parent_idx;
    link.mat_id = mat_id;
    return link;
}

void ArticulatedBody::setup() {
    int num_joints = get_num_joints();
    joint_dofs.resize(num_joints);
    joint_dof_starts.resize(num_joints + 1);
    parents.resize(num_joints);

    for (int i = 0; i < num_joints; i++) {
        auto& link = links[i];
        auto& joint = joints[i];
        // Find the unit screw axis of the current joint
        switch (joint.type) {
            case JointType::Revolute: case JointType::Prismatic: {
                joint_dofs[i] = 1;
            } break;
            case JointType::Spherical:{
                joint_dofs[i] = 3;
            } break;
        }
    }
    joint_dof_starts[0] = 0;
    for (int i = 1; i <= num_joints; i++) {
        joint_dof_starts[i] = joint_dof_starts[i-1] + joint_dofs[i-1];
    }
    num_dofs = joint_dof_starts[num_joints];

    for (int i = 0; i < num_joints; i++) {
        parents[i] = links[i].parent_idx;
    }
}
