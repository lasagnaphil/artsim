//
// Created by Phillip Chang on 2020/09/26.
//

#ifndef ARTSIM_ARTICULATION_RENDER_H
#define ARTSIM_ARTICULATION_RENDER_H

#include <artsim/artsim.h>
#include <artsim/example_articulations.h>
#include <raylib.h>
#include <rlgl.h>

Vector3 glm_to_ray(glm::vec3 v) {
    return Vector3{v.x, v.y, v.z};
}

void render_articulation(const artsim::ArticulationState& state) {
    for (int i = 0; i < state.num_joints; i++) {
        glm::vec3 pos = state.T_global[i].v;
        glm::quat rot = state.T_global[i].q;
        glm::vec3 w = artsim::log(rot);
        float angle = glm::length(w);
        glm::vec3 axis = glm::normalize(w);

        rlPushMatrix();
        rlRotatef(angle, axis.x, axis.y, axis.z);
        rlTranslatef(pos.x, pos.y, pos.z);

        artsim::Shape shape = state.art->links[i].shape;

        switch(shape.type) {
            case artsim::Shape::Type::Sphere: {
                DrawSphere(glm_to_ray(pos), shape.sphere.radius, RED);
            } break;
            case artsim::Shape::Type::Box: {
                DrawCube(glm_to_ray(pos), shape.box.size.x, shape.box.size.y, shape.box.size.z, RED);
            } break;
        }

        rlPopMatrix();
    }
}


#endif //ARTSIM_ARTICULATION_RENDER_H
