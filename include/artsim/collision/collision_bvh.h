//
// Created by lasagnaphil on 21. 9. 23..
//

#ifndef EOS_SCAN_TO_HUMAN_COLLISION_BVH_H
#define EOS_SCAN_TO_HUMAN_COLLISION_BVH_H

#include <artsim/types.h>
#include <artsim/core/arena.h>
#include <artsim/math/box.h>
#include <artsim/collision/collision_object.h>

namespace artsim {

class CollisionBVH {
public:
    using AABB = glmx::tbox<3, real>;
private:
    struct Node {
        AABB aabb;
        int left_id = -1, right_id = -1;
        Id<CollisionObject> obj_id;
    };
    std::vector<Node> nodes;

public:
    CollisionBVH() {}

    bool empty() const { return nodes.empty(); }
    void reserve(size_t num_aabbs) { nodes.reserve(2*num_aabbs); }
    void clear() { return nodes.clear(); }

    void init(const AABB* aabbs, int num_aabbs);
    AABB bounds() const { return nodes[0].aabb; }

    const std::vector<Node>& get_nodes() const { return nodes; }

    template <class Visitor>
    bool traverse(Visitor&& visitor) const {
        return traverse_children(0, visitor);
    }

    template <class PairVisitor>
    static void find_collisions(const CollisionBVH& tree1, const CollisionBVH& tree2, PairVisitor&& visitor);

private:
    void create_children(int node_id, std::vector<int>& queue,
                         const std::vector<AABB>& leaves, const std::vector<glm::rvec3>& centroids);

    template <class Visitor>
    bool traverse_children(int node_id, Visitor&& visitor) const;

    template <class PairVisitor>
    static void find_collisions(const CollisionBVH& tree1, const CollisionBVH& tree2, int node1_id, int node2_id,
                                PairVisitor&& visitor);
};

}

#endif //EOS_SCAN_TO_HUMAN_COLLISION_BVH_H
