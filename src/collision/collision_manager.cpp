//
// Created by lasagnaphil on 21. 9. 23..
//

#include <artsim/collision/collision_manager.h>

#include <fmt/core.h>

namespace artsim {

CollisionShape CollisionManager::make_ground_shape() {
    CollisionShape shape;
    shape.type = CollisionShape::Type::Ground;
    shape.scale = glm::rvec3(1);
    return shape;
}

CollisionShape CollisionManager::make_box_shape(glm::vec3 size) {
    CollisionShape shape;
    shape.type = CollisionShape::Type::Box;
    shape.scale = size;
    return shape;
}

CollisionShape CollisionManager::make_sphere_shape(real radius) {
    CollisionShape shape;
    shape.type = CollisionShape::Type::Sphere;
    shape.scale = glm::rvec3(radius, radius, radius);
    return shape;
}

CollisionShape CollisionManager::make_mesh_shape_bvh(const char* objfile, glm::rvec3 scale) {
    CollisionShape shape;
    shape.type = CollisionShape::Type::Mesh;
    shape.scale = scale;
    return shape;
}

CollisionShape CollisionManager::make_mesh_shape_sdf(const char* objfile, real cell_size, glm::rvec3 scale) {
    CollisionShape shape;
    shape.type = CollisionShape::Type::Mesh;
    shape.scale = scale;
    return shape;
}

void CollisionManager::init_bvh() {
    int num_aabbs = objects.size();
    nodes.clear();
    nodes.reserve(2*num_aabbs);
    nodes.push_back(BVHNode{});

    std::vector<Id<CollisionObject>> queue(num_aabbs);
    objects.enumerate_id_val([&](int i, Id<CollisionObject> id, const CollisionObject& obj) {
        nodes[0].aabb.extend(obj.bounds);
        queue.push_back(id);
    });

    create_children(0, queue);
}

void CollisionManager::create_children(int node_id, std::vector<Id<CollisionObject>>& queue) {
    // fmt::print("create_children({})\n", node_id);
    BVHNode& node = nodes[node_id];
    int n_queue = queue.size();
    if (n_queue == 0) {
        fmt::print("Error in BVH::create_children(): empty queue\n");
        exit(EXIT_FAILURE);
    }
    if (n_queue == 1) {
        auto id = queue[0];
        node.obj_id = id;
        node.aabb = objects.get(id)->bounds;
        return;
    }

    // Compute the splitting plane
    AABB tempAABB;
    for (auto& id : queue) {
        tempAABB.extend(objects.get(id)->bounds_center);
    }
    auto sizes = tempAABB.size();
    int split = 0;
    if (sizes[1] >= sizes[0] && sizes[1] >= sizes[2]) { split = 1; }
    else if (sizes[2] >= sizes[0] && sizes[2] >= sizes[1]) { split = 2; }

    // If two elements, make left and right
    if (n_queue == 2) {
        node.left_id = nodes.size();
        node.right_id = nodes.size() + 1;
        BVHNode left_node, right_node;
        auto id0 = queue[0];
        auto id1 = queue[1];
        auto& obj0 = *objects.get(id0);
        auto& obj1 = *objects.get(id1);

        if (obj0.bounds_center[split] < obj0.bounds_center[split]) {
            left_node.obj_id = id0;
            left_node.aabb = obj0.bounds;
            right_node.obj_id = id1;
            right_node.aabb = obj1.bounds;
        } else {
            left_node.obj_id = id1;
            left_node.aabb = obj1.bounds;
            right_node.obj_id = id0;
            right_node.aabb = obj0.bounds;
        }
        nodes.push_back(left_node);
        nodes.push_back(right_node);
        return;
    }

    // Split the queue into left and right
    real center = tempAABB.center()[split];
    // fmt::print("split on axis {} with center {}\n", split, center);
    AABB left_aabb, right_aabb;
    std::vector<Id<CollisionObject>> left_queue, right_queue;
    for (auto id : queue) {
        auto& obj = *objects.get(id);
        if (obj.bounds_center[split] < center) {
            left_queue.push_back(id);
            left_aabb.extend(obj.bounds);
        }
        else {
            right_queue.push_back(id);
            right_aabb.extend(obj.bounds);
        }
    }

    // This could possibly happen if the geometry left in the queue are all the same.
    // When this happens, just split the right queue in half and share.
    if (left_queue.size() == 0) {
        int N = right_queue.size();
        for (int i = 0; i < N/2; i++) {
            left_queue.push_back(right_queue[N-1-i]);
        }
        for (int i = 0; i < N/2; i++) {
            right_queue.pop_back();
        }
    }

    // This should not happen!
    if (right_queue.size() == 0) {
        fmt::print("CollisionBVH::init() error: problem splitting geometry\n");
        exit(EXIT_FAILURE);
    }

    // Create the left child
    node.left_id = nodes.size();
    BVHNode left_node;
    left_node.aabb = left_aabb;
    nodes.push_back(left_node);
    create_children(node.left_id, left_queue);

    // Create the right child
    node.right_id = nodes.size();
    BVHNode right_node;
    right_node.aabb = right_aabb;
    nodes.push_back(right_node);
    create_children(node.right_id, right_queue);
}

}