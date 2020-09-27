//
// Created by Phillip Chang on 2020/09/26.
//

#ifndef ARTSIM_ARTICULATION_RENDER_H
#define ARTSIM_ARTICULATION_RENDER_H

#include <artsim/artsim.h>
#include <artsim/example_articulations.h>
#include <raylib.h>
#include <rlgl.h>
#include <glm/gtc/type_ptr.hpp>

Vector3 glm_to_ray(glm::vec3 v) {
    return Vector3{v.x, v.y, v.z};
}

void render_articulation(const artsim::ArticulationState& state) {
    for (int i = 0; i < state.num_joints; i++) {
        glm::mat4 model_mat = artsim::mat4_cast(state.T_global[i]);
        rlPushMatrix();
        rlMultMatrixf(glm::value_ptr(model_mat));

        artsim::Shape shape = state.art->links[i].shape;
        switch(shape.type) {
            case artsim::Shape::Type::Sphere: {
                DrawSphere(Vector3Zero(), shape.sphere.radius, RED);
            } break;
            case artsim::Shape::Type::Box: {
                DrawCube(Vector3Zero(), shape.box.size.x, shape.box.size.y, shape.box.size.z, RED);
            } break;
        }

        rlPopMatrix();
    }
}


#endif //ARTSIM_ARTICULATION_RENDER_H
