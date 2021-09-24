//
// Created by lasagnaphil on 21. 9. 23..
//

#ifndef EOS_SCAN_TO_HUMAN_COLLISION_OBJECT_H
#define EOS_SCAN_TO_HUMAN_COLLISION_OBJECT_H

#include <artsim/material.h>
#include <artsim/math/box.h>
#include <artsim/collision/collision_shape.h>

namespace artsim {

struct CollisionObject {
    Id<CollisionShape> shape_id;
    Id<Material> mat_id;
    glmx::rtransform world_trans;
    glmx::tbox<3, real> bounds;
    glm::rvec3 bounds_center;
};

}

#endif //EOS_SCAN_TO_HUMAN_COLLISION_OBJECT_H
