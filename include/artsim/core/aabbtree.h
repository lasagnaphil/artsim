// BVH data structure ported from https://github.com/mattoverby/mclscene

#ifndef EOS_SCAN_TO_HUMAN_AABBTREE_H
#define EOS_SCAN_TO_HUMAN_AABBTREE_H

#include <artsim/core/aabbtree_visitor.h>
#include <artsim/math/box.h>

#include <glm/gtx/string_cast.hpp>

#include <numeric>
#include <fmt/core.h>

namespace artsim {

template <class T>
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

    template <int PDIM>
    void init(const glm::rvec3* vertices, const glm::vec<PDIM, int>* indices, int num_prims);
    void init(const AABB* aabbs, int num_aabbs);
    AABB bounds() { return nodes[0].aabb; }

    bool traverse(AABBTreeVisitor<T>& visitor) {
        return traverse_children(0, visitor);
    }
private:
    void create_children(int node_id, std::vector<int>& queue,
                         const std::vector<AABB>& leaves, const std::vector<glm::rvec3>& centroids);

    bool traverse_children(int node_id, AABBTreeVisitor<T>& visitor);
};

template <class T>
template <int PDIM>
void AABBTree<T>::init(const glm::rvec3* vertices, const glm::vec<PDIM, int>* indices, int num_prims) {
    static_assert(PDIM > 0);
    nodes.clear();
    nodes.reserve(2*num_prims);
    nodes.push_back(Node{});
    std::vector<AABB> leaf_aabb(num_prims);
    std::vector<glm::rvec3> leaf_centroids(num_prims, glm::rvec3(0));
    for (int pidx = 0; pidx < num_prims; pidx++) {
        auto idx = indices[pidx];
        for (int k = 0; k < PDIM; k++) {
            auto p = vertices[idx[k]];
            leaf_aabb[pidx].extend(p);
            leaf_centroids[pidx] += p;
        }
        leaf_centroids[pidx] /= PDIM;
    }
    for (int pidx = 0; pidx < num_prims; pidx++) {
        nodes[0].aabb.extend(leaf_aabb[pidx]);
    }

    std::vector<int> queue(num_prims);
    std::iota(queue.begin(), queue.end(), 0);
    create_children(0, queue, leaf_aabb, leaf_centroids);
}

template<class T>
void AABBTree<T>::init(const AABBTree::AABB* aabbs, int num_aabbs) {
    nodes.clear();
    nodes.reserve(2*num_aabbs);
    nodes.push_back(Node{});
    std::vector<AABB> leaf_aabb(num_aabbs);
    std::vector<glm::rvec3> leaf_centroids(num_aabbs, glm::rvec3(0));
    for (int i = 0; i < num_aabbs; i++) {
        auto& aabb = aabbs[i];
        leaf_aabb[i].extend(aabb);
        leaf_centroids[i] = aabb.center();
    }
    for (int i = 0; i < num_aabbs; i++) {
        nodes[0].aabb.extend(leaf_aabb[i]);
    }

    std::vector<int> queue(num_aabbs);
    std::iota(queue.begin(), queue.end(), 0);
    create_children(0, queue, leaf_aabb, leaf_centroids);
}

template<class T>
void AABBTree<T>::create_children(int node_id, std::vector<int>& queue, const std::vector<AABB>& leaves,
                                  const std::vector<glm::rvec3>& centroids) {
    // fmt::print("create_children({})\n", node_id);
    Node& node = nodes[node_id];
    int n_queue = queue.size();
    if (n_queue == 0) {
        fmt::print("Error in BVH::create_children(): empty queue\n");
        exit(EXIT_FAILURE);
    }
    if (n_queue == 1) {
        int qidx = queue[0];
        node.prim_id = qidx;
        node.aabb = leaves[qidx];
        return;
    }

    // Compute the splitting plane
    AABB tempAABB;
    for (int i = 0; i < n_queue; ++i) { tempAABB.extend(centroids[queue[i]]); }
    auto sizes = tempAABB.size();
    int split = 0;
    if (sizes[1] >= sizes[0] && sizes[1] >= sizes[2]) { split = 1; }
    else if (sizes[2] >= sizes[0] && sizes[2] >= sizes[1]) { split = 2; }

    // If two elements, make left and right
    if (n_queue == 2) {
        node.left_id = nodes.size();
        node.right_id = nodes.size() + 1;
        Node left_node, right_node;
        int idx0 = queue[0];
        int idx1 = queue[1];
        const glm::rvec3& cent0 = centroids[idx0];
        const glm::rvec3& cent1 = centroids[idx1];

        if (cent0[split] < cent1[split]) {
            left_node.prim_id = idx0;
            left_node.aabb = leaves[idx0];
            right_node.prim_id = idx1;
            right_node.aabb = leaves[idx1];
        } else {
            left_node.prim_id = idx1;
            left_node.aabb = leaves[idx1];
            right_node.prim_id = idx0;
            right_node.aabb = leaves[idx0];
        }
        nodes.push_back(left_node);
        nodes.push_back(right_node);
        return;
    }

    // Split the queue into left and right
    T center = tempAABB.center()[split];
    // fmt::print("split on axis {} with center {}\n", split, center);
    AABB left_aabb, right_aabb;
    std::vector<int> left_queue, right_queue;
    for (int i = 0; i < n_queue; ++i) {
        int idx = queue[i];
        const glm::rvec3& cent = centroids[idx];
        if (cent[split] < center) {
            // fmt::print("insert on left {}\n", glm::to_string(cent));
            left_queue.push_back(idx);
            left_aabb.extend(leaves[idx]);
        } else {
            // fmt::print("insert on right {}\n", glm::to_string(cent));
            right_queue.push_back(idx);
            right_aabb.extend(leaves[idx]);
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

template<class T>
bool AABBTree<T>::traverse_children(int node_id, AABBTreeVisitor<T>& visitor) {
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
