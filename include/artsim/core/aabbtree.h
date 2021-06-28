// BVH data structure ported from https://github.com/mattoverby/mclscene

#ifndef EOS_SCAN_TO_HUMAN_AABBTREE_H
#define EOS_SCAN_TO_HUMAN_AABBTREE_H

#include <artsim/core/aabbtree_visitor.h>
#include <artsim/math/box.h>

#include <numeric>
#include <fmt/core.h>

namespace artsim {

template <class T, int PDIM>
class AABBTree {
public:
    using AABB = glmx::tbox<3, T>;
private:
    struct Node {
        AABB aabb;
        int left_id = -1, right_id = -1;
        int prim_id = -1;
    };
    std::vector<Node> nodes;

public:
    AABBTree() {
    }

    void init(const glm::rvec3* vertices, const int* indices, int num_prims);

    bool traverse(AABBTreeVisitor<T>& visitor) {
        return traverse_children(0, visitor);
    }
private:
    void create_children(int node_id, std::vector<int>& queue,
                         const std::vector<AABB>& leaves, const std::vector<glm::rvec3>& centroids);

    bool traverse_children(int node_id, AABBTreeVisitor<T>& visitor);
};

template<class T, int PDIM>
void AABBTree<T, PDIM>::init(const glm::rvec3* vertices, const int* indices, int num_prims) {
    nodes.clear();
    nodes.push_back(Node{});
    std::vector<AABB> leaf_aabb;
    std::vector<glm::rvec3> leaf_centroids;
    for (int pidx = 0; pidx < num_prims; pidx++) {
        for (int k = 0; k < PDIM; k++) {
            int prim_id = indices[pidx*PDIM + k];
            auto p = vertices[prim_id] ;
            leaf_aabb.extend(p);
            leaf_centroids[pidx] += p;
        }
        leaf_centroids[pidx] /= PDIM;
    }
    for (int pidx = 0; pidx < num_prims; pidx++) {
        nodes[0].extend(leaf_aabb[pidx]);
    }

    std::vector<int> queue(num_prims);
    std::iota(queue.begin(), queue.end(), 0);
    create_children(0, queue, leaf_aabb, leaf_centroids);
}

template<class T, int PDIM>
void AABBTree<T, PDIM>::create_children(int node_id, std::vector<int>& queue, const std::vector<AABB>& leaves,
                                        const std::vector<glm::rvec3>& centroids) {
    Node& node = nodes[node_id];
    int n_queue = queue.size();
    if (n_queue == 0) {
        fmt::print("Error in BVH::create_children(): empty queue\n");
        exit(EXIT_FAILURE);
    }
    if (n_queue == 1) {
        int qidx = queue[0];
        node.prim = qidx;
        node.aabb = leaves[qidx];
        return;
    }

    // Compute the splitting plane
    AABB tempAABB;
    for (int i = 0; i < n_queue; ++i) { tempAABB.extend(centroids[queue[i]]); }
    auto sizes = tempAABB.sizes();
    int split = 0;
    if (sizes[1] >= sizes[0] && sizes[1] >= sizes[2]) { split = 1; }
    else if (sizes[2] >= sizes[0] && sizes[2] >= sizes[1]) { split = 2; }

    // If two elements, make left and right
    if (n_queue == 2) {
        node.left_id = node.size();
        node.right_id = node.size() + 1;
        Node left_node, right_node;
        int idx0 = queue[0];
        int idx1 = queue[1];
        const glm::rvec3& cent0 = centroids[idx0];
        const glm::rvec3& cent1 = centroids[idx1];

        if (cent0[split] < cent1[split]) {
            left_node.prim = idx0;
            left_node.aabb = leaves[idx0];
            right_node.prim = idx1;
            right_node.aabb = leaves[idx1];
        } else {
            left_node.prim = idx1;
            left_node.aabb = leaves[idx1];
            right_node.prim = idx0;
            right_node.aabb = leaves[idx0];
        }
        node.push_back(left_node);
        node.push_back(right_node);
        return;
    }

    // Split the queue into left and right
    T center = tempAABB.center()[split];
    AABB left_aabb, right_aabb;
    std::vector<int> left_queue, right_queue;
    for (int i = 0; i < n_queue; ++i) {
        int idx = queue[i];
        const glm::rvec3& cent = centroids[idx];
        if (cent[split] < center) {
            left_queue.push_back(idx);
            left_aabb.extend(leaves[idx]);
        } else {
            right_queue.push_back(idx);
            right_aabb.extend(leaves[idx]);
        }
    }

    if (left_queue.size() == 0 || right_queue.size() == 0) {
        fmt::print("AABBTree::init() error: problem splitting geometry\n");
        exit(EXIT_FAILURE);
    }

    // Create the left child
    node.left_id = nodes.size();
    Node left_node;
    left_node.aabb = left_aabb;
    nodes.push_back(left_node);
    create_children(node.left_id, left_queue, leaves, centroids);

    // Create the right child
    node.right_id = nodes.size();
    Node right_node;
    right_node.aabb = right_aabb;
    nodes.push_back(right_node);
    create_children(node.right_id, right_queue, leaves, centroids);
}

template<class T, int PDIM>
bool AABBTree<T, PDIM>::traverse_children(int node_id, AABBTreeVisitor<T>& visitor) {
    auto& node = nodes[node_id];
    if (!visitor.hit_aabb(node.aabb)) {
        return false;
    }
    if (node.left_id != -1 && node.right_id != -1) {
        auto& left_node = nodes[node.left_id];
        auto& right_node = nodes[node.right_id];
        bool check_left_first = visitor.check_left_first(left_node, right_node);
        if (check_left_first) {
            if (traverse_children(node.left_id, visitor)) return true;
            else return traverse_children(node.right_id, visitor);
        }
        else {
            if (traverse_children(node.right_id, visitor)) return true;
            else return traverse_children(node.left_id, visitor);
        }
    }
    if (node.left_id != -1) return traverse_children(node.left_id, visitor);
    if (node.right_id != -1) return traverse_children(node.right_id, visitor);
    if (node.prim_id == -1) {
        fmt::print("AABBTree::traverse() error: leaf has no primitive\n");
        exit(EXIT_FAILURE);
    }
    return visitor.hit_prim(node.prim_id);
}

}

#endif //EOS_SCAN_TO_HUMAN_AABBTREE_H
