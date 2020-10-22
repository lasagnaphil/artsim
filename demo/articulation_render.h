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
                DrawSphereWires(Vector3Zero(), shape.box.size.x, shape.box.size.y, shape.box.size.z, GRAY);
            } break;
            case artsim::Shape::Type::Box: {
                DrawCube(Vector3Zero(), shape.box.size.x, shape.box.size.y, shape.box.size.z, RED);
                DrawCubeWires(Vector3Zero(), shape.box.size.x, shape.box.size.y, shape.box.size.z, GRAY);
            } break;
            default: {}
        }

        rlPopMatrix();
    }
    for (int c = 0; c < state.contact_points.size(); c++) {
        glm::tvec3<T> normal = state.contact_normals[c];
        const artsim::ContactPoint& cpoint = state.contact_points[c];

        glm::tvec3<T> contact_Ey = glm::normalize(glm::cross(normal, artsim::Ez<T>()));
        glm::tmat3x3<T> contact_mat(artsim::Ez<T>(), contact_Ey, normal);

        DrawLine3D(glm_to_ray(cpoint.pos), glm_to_ray(cpoint.pos + 0.1f * glm::vec3(contact_mat * normal)), GREEN);
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
