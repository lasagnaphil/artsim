//
// Created by lasagnaphil on 20. 10. 22..
//

#ifndef ARTSIM_COLLISION_H
#define ARTSIM_COLLISION_H

#include "artsim/artsim.h"
#include "artsim/dynamics.h"
#include "artsim/math/common.h"

namespace artsim {

template <class T>
void contact_points_between_art_links_and_ground(
        const ArticulatedBody &art,
        const Id<ArticulatedBody> art_id,
        const uint32_t* link_indices, uint32_t link_indices_count,
        const ttransform<T>* link_global_trans,
        OUT ContactPoint* contact_points,
        OUT uint32_t& contact_points_count) {

    std::vector<glm::tvec3<T>> cpos;
    contact_points_count = 0;

    const float epsilon = 1e-7f;
    for (uint32_t li = 0; li < link_indices_count; li++) {
        uint32_t i = link_indices[li];
        switch (art.links[i].shape.type) {
            case artsim::Shape::Type::Box: {
                glm::tvec3<T> ext = 0.5f * art.links[i].shape.box.size;
                glm::tvec3<T> p = link_global_trans[i].v;
                if (p.y*p.y > ext.x*ext.x + ext.y*ext.y + ext.z*ext.z) {
                    // early bailout for boxes that definitely doesn't collide with ground
                    break;
                }
                p = link_global_trans[i].v + link_global_trans[i].q * glm::tvec3<T>(-ext.x, -ext.y, -ext.z);
                if (p.y <= epsilon) {
                    cpos.push_back(p);
                }
                p = link_global_trans[i].v + link_global_trans[i].q * glm::tvec3<T>(-ext.x, -ext.y,  ext.z);
                if (p.y <= epsilon) {
                    cpos.push_back(p);
                }
                p = link_global_trans[i].v + link_global_trans[i].q * glm::tvec3<T>(-ext.x,  ext.y, -ext.z);
                if (p.y <= epsilon) {
                    cpos.push_back(p);
                }
                p = link_global_trans[i].v + link_global_trans[i].q * glm::tvec3<T>(-ext.x,  ext.y,  ext.z);
                if (p.y <= epsilon) {
                    cpos.push_back(p);
                }
                p = link_global_trans[i].v + link_global_trans[i].q * glm::tvec3<T>( ext.x, -ext.y, -ext.z);
                if (p.y <= epsilon) {
                    cpos.push_back(p);
                }
                p = link_global_trans[i].v + link_global_trans[i].q * glm::tvec3<T>( ext.x, -ext.y,  ext.z);
                if (p.y <= epsilon) {
                    cpos.push_back(p);
                }
                p = link_global_trans[i].v + link_global_trans[i].q * glm::tvec3<T>( ext.x,  ext.y, -ext.z);
                if (p.y <= epsilon) {
                    cpos.push_back(p);
                }
                p = link_global_trans[i].v + link_global_trans[i].q * glm::tvec3<T>( ext.x,  ext.y,  ext.z);
                if (p.y <= epsilon) {
                    cpos.push_back(p);
                }
                if (!cpos.empty()) {
                    glm::tvec3<T> cpos_avg = glm::tvec3<T>(0);
                    for (auto& pos : cpos) {
                        cpos_avg += pos;
                    }
                    cpos_avg /= cpos.size();
                    contact_points[contact_points_count++] = ContactPoint(
                            glm::vec3(cpos_avg.x, 0, cpos_avg.z), Ey<T>(), Ez<T>(), -cpos_avg.y,
                            RigidBodyOrLink::from_articulation_link(art_id, i),
                            RigidBodyOrLink::from_rigid_body(Id<RigidBody>::null()));
                }
            } break;
            case artsim::Shape::Type::Sphere: {
                glm::vec3 p = link_global_trans[i].v;
                float d = p.y - art.links[i].shape.sphere.radius;
                if (d <= 0.0f) {
                    contact_points[contact_points_count++] = ContactPoint(
                            glm::vec3(p.x, 0, p.z), Ey<T>(), Ez<T>(), -d,
                            RigidBodyOrLink::from_articulation_link(art_id, i),
                            RigidBodyOrLink::from_rigid_body(Id<RigidBody>::null()));
                }
            } break;
        }
    }
}

}


#endif //ARTSIM_COLLISION_H
