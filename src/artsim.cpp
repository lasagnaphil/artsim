//
// Created by lasagnaphil on 20. 5. 19..
//

#include "artsim/artsim.h"
#include "artsim/math/se3.h"

#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>

#include <queue>

using namespace artsim;
using namespace glmx;

real CollisionShape::mass(real density) {
    switch (type) {
        case Type::Box: return density * box.size.x * box.size.y * box.size.z;
        case Type::Sphere: return real(4.0 / 3.0) * glm::pi<real>() * sphere.radius * sphere.radius * sphere.radius;
        case Type::Mesh: {
            real V = 0;
            auto& obj = *mesh.obj;
            for (auto tri : obj.triangle_vertices) {
                auto v0 = obj.vertices[tri[0]];
                auto v1 = obj.vertices[tri[1]];
                auto v2 = obj.vertices[tri[2]];
                V += glm::determinant(glm::mat3(v0, v1, v2));
            }
            V *= (density / 6);
            V = glm::abs(V);
            return V;
        }
        default: return real(0);
    }
}

tsmat3x3<real> CollisionShape::inertia(real density) {
    switch (type) {
        case Type::Box: {
            const glm::tvec3<real>& s = box.size;
            glm::vec3 I = mass(density) * glm::tvec3<real>(s.y*s.y + s.z*s.z, s.z*s.z + s.x*s.x, s.x*s.x + s.y*s.y) / real(12);
            return tsmat3x3<real>(I.x, I.y, I.z, 0, 0, 0);
        }
        case Type::Sphere: {
            real r = sphere.radius;
            glm::vec3 I = real(0.4) * mass(density) * glm::tvec3<real>(r*r);
            return tsmat3x3<real>(I.x, I.y, I.z, 0, 0, 0);
        }
        case Type::Mesh: {
            // TODO: Calculate proper inertia
            return tsmat3x3<real>(1, 1, 1, 0, 0, 0);
        }
        default: return tsmat3x3<real>(0, 0, 0, 0, 0, 0);
    }
}

CollisionShape CollisionShape::make_ground() {
    CollisionShape shape;
    shape.type = CollisionShape::Type::Ground;
    shape.bt_shape = new btStaticPlaneShape(btVector3(0, 0, 0), 0);
    return shape;
}

CollisionShape CollisionShape::make_box(glm::vec3 size) {
    CollisionShape shape;
    shape.type = CollisionShape::Type::Box;
    shape.box.size = size;
    shape.bt_shape = new btBoxShape(btconv(real(0.5) * shape.box.size));
    return shape;
}

CollisionShape CollisionShape::make_sphere(real radius) {
    CollisionShape shape;
    shape.type = CollisionShape::Type::Sphere;
    shape.sphere.radius = radius;
    shape.bt_shape = new btSphereShape(shape.sphere.radius);
    return shape;
}

CollisionShape CollisionShape::make_mesh(OBJFile* obj) {
    CollisionShape shape;
    shape.type = CollisionShape::Type::Mesh;
    shape.mesh.obj = obj;
    // TODO: Create Bullet ConcaveMesh
    shape.bt_shape = nullptr;
    return shape;
}
Link Link::create(const tsmat3x3<real>& inertia, real mass, CollisionShape shape,
                  ttransform<real> local_joint_pose, ttransform<real> local_link_pose,
                  int parent_idx, Id<Material> mat_id) {
    Link link;
    link.inertia = inertia;
    link.mass = mass;
    link.col_shape = shape;
    link.local_joint_pose = local_joint_pose;
    link.local_link_pose = local_link_pose;
    link.parent_idx = parent_idx;
    link.mat_id = mat_id;
    auto I0 = tspmat<real>(tsmat3x3<real>(link.inertia), glm::tvec3<real>(0), link.mass);
    link.I_j = inv_transform(I0, ttransform<real>(inverse(link.local_link_pose)));
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

    // TODO: initialize btCollisionWorld outside this function
    auto bt_collision_config = new btDefaultCollisionConfiguration;
    auto bt_dispatcher = new btCollisionDispatcher(bt_collision_config);
    auto bt_broadphase = new btDbvtBroadphase;
    bt_collision_world = new btCollisionWorld(bt_dispatcher, bt_broadphase, bt_collision_config);

    auto bt_plane_col = new btCollisionObject;
    bt_plane_col->setCollisionShape(new btStaticPlaneShape(btVector3(0, 1, 0), 0));
    bt_plane_col->setWorldTransform(btTransform::getIdentity());
    bt_plane_col->setUserIndex(0);
    bt_plane_col->setUserIndex2(0);
    bt_collision_world->addCollisionObject(bt_plane_col, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);

    for (int i = 0; i < links.size(); i++) {
        // TODO: Allocate these from a separate array!
        // TODO: Set body_id with current articulation id
        auto col_shape = links[i].col_shape;
        if (col_shape.type != CollisionShape::Type::Mesh) {
            BodyId body_id = BodyId::from_articulation_link({}, i);
            btCollisionObject* col_obj = new btCollisionObject;
            col_obj->setCollisionShape(links[i].col_shape.bt_shape);
            col_obj->setUserIndex(body_id.index);
            col_obj->setUserIndex2(body_id.generation);
            bt_collision_world->addCollisionObject(col_obj, 0b1000000, ~0b1000000);
            links[i].bt_collision_object = col_obj;
        }
    }
}
