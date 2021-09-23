//
// Created by lasagnaphil on 21. 9. 23..
//

#ifndef EOS_SCAN_TO_HUMAN_COLLISION_OBJECT_H
#define EOS_SCAN_TO_HUMAN_COLLISION_OBJECT_H

namespace artsim {

struct CollisionObject {
    Id<CollisionShape> shape_id;
    Id<Material> mat_id;
    glmx::rtransform world_trans;
};

}

#endif //EOS_SCAN_TO_HUMAN_COLLISION_OBJECT_H
