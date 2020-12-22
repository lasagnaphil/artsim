//
// Created by lasagnaphil on 20. 5. 19..
//

#include "artsim/artsim.h"
#include "artsim/math/se3.h"

#include <queue>

using namespace artsim;

float Shape::mass(float density) {
    switch (type) {
        case Type::Box: return density * box.size.x * box.size.y * box.size.z;
        case Type::Sphere: return 4.f / 3.f * glm::pi<float>() * sphere.radius * sphere.radius * sphere.radius;
        default: return 0;
    }
}

smat3x3 Shape::inertia(float density) {
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
    return smat3x3(I.x, I.y, I.z, 0, 0, 0);
}

Shape Shape::make_ground() {
    Shape shape;
    shape.type = Shape::Type::Ground;
    return shape;
}
Shape Shape::make_box(glm::vec3 size) {
    Shape shape;
    shape.type = Shape::Type::Box;
    shape.box.size = size;
    return shape;
}

Shape Shape::make_sphere(float radius) {
    Shape shape;
    shape.type = Shape::Type::Sphere;
    shape.sphere.radius = radius;
    return shape;
}

Link Link::create(const smat3x3& inertia, float mass, Shape shape, transform local_joint_pose, transform local_link_pose,
                  int parent_idx, Id<Material> mat_id) {
    Link link;
    link.inertia = inertia;
    link.mass = mass;
    link.shape = shape;
    link.local_joint_pose = local_joint_pose;
    link.local_link_pose = local_link_pose;
    link.parent_idx = parent_idx;
    link.mat_id = mat_id;
    return link;
}

void ArticulatedBody::setup() {
    int num_joints = get_num_joints();
    joint_pos_dofs.resize(num_joints);
    joint_pos_dof_starts.resize(num_joints + 1);
    joint_vel_dofs.resize(num_joints);
    joint_vel_dof_starts.resize(num_joints + 1);
    parents.resize(num_joints);

    for (int i = 0; i < num_joints; i++) {
        auto& link = links[i];
        auto& joint = joints[i];
        // Find the unit screw axis of the current joint
        joint_pos_dofs[i] = joint.pos_dof();
        joint_vel_dofs[i] = joint.vel_dof();
    }
    joint_pos_dof_starts[0] = 0;
    for (int i = 1; i <= num_joints; i++) {
        joint_pos_dof_starts[i] = joint_pos_dof_starts[i-1] + joint_pos_dofs[i-1];
    }
    num_pos_dofs = joint_pos_dof_starts[num_joints];
    joint_vel_dof_starts[0] = 0;
    for (int i = 1; i <= num_joints; i++) {
        joint_vel_dof_starts[i] = joint_vel_dof_starts[i-1] + joint_vel_dofs[i-1];
    }
    num_vel_dofs = joint_vel_dof_starts[num_joints];

    for (int i = 0; i < num_joints; i++) {
        parents[i] = links[i].parent_idx;
    }
    children_buffer.reserve(links.size()-1);
    children_buffer_starts.resize(links.size()+1);
    for (int i = 0; i < links.size(); i++) {
        children_buffer_starts[i] = children_buffer.size();
        for (int j = 1; j < links.size(); j++) {
            if (i == parents[j]) {
                children_buffer.push_back(j);
            }
        }
    }
    children_buffer_starts[links.size()] = children_buffer.size();
    std::queue<uint32_t> queue;
    queue.push(0);
    while (!queue.empty()) {
        uint32_t i = queue.front();
        queue.pop();
        uint32_t num_children = get_num_children(i);
        bfs_iteration_order.push_back(i);

        const uint32_t* i_children = get_children(i);
        for (uint32_t c = 0; c < num_children; c++) {
            queue.push(i_children[c]);
        }
    }
}
