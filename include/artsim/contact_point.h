//
// Created by lasagnaphil on 8/21/21.
//

#ifndef ARTSIM_CONTACT_POINT_H
#define ARTSIM_CONTACT_POINT_H

#include <artsim/art_body.h>
#include <artsim/rigid_body.h>

namespace artsim {

struct BodyId {
    /*
     * Memory layout:
     *
    bool is_art: 1;
    union {
        uint32_t rb_idx: 31;
        uint32_t art_idx: 31;
    };
     */
    uint32_t index;
    uint32_t generation;

    friend bool operator==(BodyId id1, BodyId id2);
    friend bool operator!=(BodyId id1, BodyId id2);

    static BodyId from_articulated_body(Id<ArticulatedBody> id) {
        auto [index, generation] = id.to_int32s();
        BodyId body_id;
        body_id.index = 0x80000000 | index;
        body_id.generation = generation;
        return body_id;
    }
    static BodyId from_rigid_body(Id<RigidBody> id) {
        auto [index, generation] = id.to_int32s();
        BodyId body_id;
        body_id.index = index;
        body_id.generation = generation;
        return body_id;
    }
    Id<ArticulatedBody> get_art_id() const {
        if (!is_articulation()) return Id<ArticulatedBody>::null();
        uint32_t art_index = index & 0x7fffffff;
        return Id<ArticulatedBody>::from_int32s(art_index, generation);
    }
    Id<RigidBody> get_rigid_body_id() const {
        if (is_articulation()) return Id<RigidBody>::null();
        return Id<RigidBody>::from_int32s(index, generation);
    }

    bool is_rigid_body() const {
        return (index & 0x80000000) == 0;
    }
    bool is_articulation() const {
        return (index & 0x80000000) != 0;
    }
};

inline bool operator==(BodyId id1, BodyId id2) {
    return id1.index == id2.index && id1.generation == id2.generation;
}
inline bool operator!=(BodyId id1, BodyId id2) {
    return id1.index != id2.index || id1.generation != id2.generation;
}

struct BodyLinkId {
    /*
     * Memory layout:
     *
    bool is_art: 1;
    union {
        uint32_t rigid_body_idx: 31;
        struct {
            uint16_t art_idx: 15;
            uint16_t art_body_idx: 16;
        };
    };
     */
    uint32_t index;
    uint32_t generation;

    friend bool operator==(BodyLinkId id1, BodyLinkId id2);
    friend bool operator!=(BodyLinkId id1, BodyLinkId id2);

    static BodyLinkId from_articulation_link(Id<ArticulatedBody> id, uint16_t link_idx) {
        auto [index, generation] = id.to_int32s();
        BodyLinkId body_id;
        body_id.index = 0x80000000 | ((index & 0x0000ffff) << 16) | link_idx;
        body_id.generation = generation;
        return body_id;
    }
    static BodyLinkId from_rigid_body(Id<RigidBody> id) {
        auto [index, generation] = id.to_int32s();
        BodyLinkId body_id;
        body_id.index = index;
        body_id.generation = generation;
        return body_id;
    }
    static BodyLinkId from_ground() {
        return {0, 0};
    }
    std::pair<Id<ArticulatedBody>, uint32_t> get_articulation_id() const {
        if (!is_articulation()) return {Id<ArticulatedBody>::null(), 0};
        uint32_t art_index = (index & 0x7fff0000) >> 16;
        uint32_t art_body_index = index & 0x0000ffff;
        return {Id<ArticulatedBody>::from_int32s(art_index, generation), art_body_index};
    }
    Id<RigidBody> get_rigid_body_id() const {
        if (is_articulation()) return Id<RigidBody>::null();
        return Id<RigidBody>::from_int32s(index, generation);
    }

    BodyId get_body_id() const {
        BodyId bid;
        if (is_articulation()) {
            uint32_t art_index = (index & 0x7fff0000) >> 16;
            bid.index = art_index | 0x80000000;
        }
        else {
            bid.index = index;
        }
        bid.generation = generation;
        return bid;
    }

    bool is_rigid_body() const {
        return (index & 0x80000000) == 0;
    }
    bool is_articulation() const {
        return (index & 0x80000000) != 0;
    }
};

inline bool operator==(BodyLinkId id1, BodyLinkId id2) {
    return id1.index == id2.index && id1.generation == id2.generation;
}
inline bool operator!=(BodyLinkId id1, BodyLinkId id2) {
    return id1.index != id2.index || id1.generation != id2.generation;
}

struct ContactPoint {
    btPersistentManifold* bt_manifold;
    btManifoldPoint* bt_manifold_point;
    glm::tvec3<real> pos;
    glm::tvec3<real> normal;
    glm::tvec3<real> tangent1;
    glm::tvec3<real> tangent2;
    glm::tvec3<real> lam;
    real distance;
    real area;
    BodyLinkId body1_id;
    BodyLinkId body2_id;
    glmx::rtransform body1_rel_trans;
    glmx::rtransform body2_rel_trans;
};

struct Frame {
    BodyLinkId body_id;
    glmx::rtransform T_local;

    static Frame from_articulation(Id<ArticulatedBody> id, int link_idx,
                                   const glmx::rtransform& T_local) {
        return {BodyLinkId::from_articulation_link(id, link_idx), T_local};
    }
};

}

namespace std {
template<>
struct hash<artsim::BodyLinkId> {
    std::size_t operator()(const artsim::BodyLinkId& id) const {
        using std::hash;
        return hash<uint32_t>()(id.index) ^ (hash<uint32_t>()(id.generation) << 1);
    }
};
}

namespace std {
template<>
struct hash<artsim::BodyId> {
    std::size_t operator()(const artsim::BodyId& id) const {
        using std::hash;
        return hash<uint32_t>()(id.index) ^ (hash<uint32_t>()(id.generation) << 1);
    }
};
}
#endif //ARTSIM_CONTACT_POINT_H
