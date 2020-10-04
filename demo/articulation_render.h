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

#include <imgui.h>

Vector3 glm_to_ray(glm::vec3 v) {
    return Vector3{v.x, v.y, v.z};
}

template <class T>
void render_articulation(const artsim::ArticulationState<T>& state) {
    for (int i = 0; i < state.num_joints; i++) {
        glm::tmat4x4<T> model_mat = artsim::mat4_cast(state.T_global[i]);
        glm::mat4 model_mat_f = model_mat;
        rlPushMatrix();
        rlMultMatrixf(glm::value_ptr(model_mat_f));

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

struct ScrollingBuffer {
    int MaxSize;
    int Offset;
    ImVector<ImVec2> Data;
    ScrollingBuffer() {
        MaxSize = 2000;
        Offset  = 0;
        Data.reserve(MaxSize);
    }
    void AddPoint(float x, float y) {
        if (Data.size() < MaxSize)
            Data.push_back(ImVec2(x,y));
        else {
            Data[Offset] = ImVec2(x,y);
            Offset =  (Offset + 1) % MaxSize;
        }
    }
    void Erase() {
        if (Data.size() > 0) {
            Data.shrink(0);
            Offset  = 0;
        }
    }
};

#endif //ARTSIM_ARTICULATION_RENDER_H
