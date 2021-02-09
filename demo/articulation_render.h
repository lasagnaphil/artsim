//
// Created by Phillip Chang on 2020/09/26.
//

#ifndef ARTSIM_ARTICULATION_RENDER_H
#define ARTSIM_ARTICULATION_RENDER_H

#include <artsim/artsim.h>
#include <artsim/utils/example_articulations.h>
#include <raylib.h>
#include <rlgl.h>
#include <glm/gtc/type_ptr.hpp>

#include <imgui.h>

inline Vector3 glm_to_ray(glm::vec3 v) {
    return Vector3{v.x, v.y, v.z};
}

inline void render_articulation(const artsim::ArticulationState& state, Color color = RED) {
    for (int i = 0; i < state.num_joints; i++) {
        glm::tmat4x4<artsim::real> model_mat = glmx::mat4_cast(state.T_link_global[i]);
        glm::mat4 model_mat_f = model_mat;
        rlPushMatrix();
        rlMultMatrixf(glm::value_ptr(model_mat_f));

        artsim::Shape shape = state.art->links[i].shape;
        switch(shape.type) {
            case artsim::Shape::Type::Sphere: {
                DrawSphere(Vector3Zero(), shape.sphere.radius, color);
                DrawSphereWires(Vector3Zero(), shape.sphere.radius, 10, 10, GRAY);
            } break;
            case artsim::Shape::Type::Box: {
                DrawCube(Vector3Zero(), shape.box.size.x, shape.box.size.y, shape.box.size.z, color);
                DrawCubeWires(Vector3Zero(), shape.box.size.x, shape.box.size.y, shape.box.size.z, GRAY);
            } break;
            default: {}
        }

        rlPopMatrix();

        rlPushMatrix();

        if (!(state.art->floating && i == 0)) {
            model_mat = glmx::mat4_cast(state.T_joint_global[i]);
            model_mat_f = model_mat;
            rlMultMatrixf(glm::value_ptr(model_mat_f));

            DrawSphere(Vector3Zero(), 0.02f, GREEN);
        }

        rlPopMatrix();
    }
    for (int c = 0; c < state.contact_points.size(); c++) {
        using namespace artsim;
        auto normal = state.contact_normals[c];
        const ContactPoint& cp = state.contact_points[c];

        auto tangent_u = glmx::Ez<real>();
        auto tangent_v = glm::cross(cp.normal, tangent_u);
        auto contact_T = glmx::ttransform<real>(cp.pos, glm::tmat3x3<real>(tangent_u, tangent_v, cp.normal));

        DrawLine3D(glm_to_ray(glm::vec3(contact_T.v)),
                   glm_to_ray(glm::vec3(contact_T.v + real(1.0) * (contact_T.R * normal))),
                   GREEN);
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
