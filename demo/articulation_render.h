//
// Created by Phillip Chang on 2020/09/26.
//

#ifndef ARTSIM_ARTICULATION_RENDER_H
#define ARTSIM_ARTICULATION_RENDER_H

#include <artsim/artsim.h>
#include <artsim/utils/example_articulations.h>
#include <glm/gtc/type_ptr.hpp>
#include <imgui.h>

#include <gengine/Arena.h>
#include <gengine/PBRenderer.h>
#include <gengine/DebugRenderer.h>

using namespace artsim;

class ArticulationRender {
public:
    ArticulationRender() = default;
    ArticulationRender(ArticulatedBody* art,
                       Ref<PBRMaterial> link_mat = {}, Ref<PBRMaterial> joint_mat = {})
            : art(art), link_mat(link_mat), joint_mat(joint_mat) {

        int num_joints = art->get_num_joints();
        link_meshes.resize(num_joints);
        joint_meshes.resize(num_joints);

        for (int i = 0; i < num_joints; i++) {
            artsim::CollisionShape shape = art->links[i].col_shape;
            switch(shape.type) {
                case artsim::CollisionShape::Type::Sphere: {
                    link_meshes[i] = Mesh::makeSphere(shape.sphere.radius);
                } break;
                case artsim::CollisionShape::Type::Box: {
                    link_meshes[i] = Mesh::makeCube(shape.box.size);
                } break;
                default: {}
            }
            joint_meshes[i] = Mesh::makeSphere(0.02f);
        }

        if (!link_mat) {
            this->link_mat = std::make_shared<PBRMaterial>();
            this->link_mat->texAlbedo = Texture::fromSingleColor({0.5f, 0.0f, 0.0f});
            this->link_mat->texAO = Texture::fromSingleColor({1.0f, 0.0f, 0.0f});
            this->link_mat->texMetallic = Texture::fromSingleColor({0.5f, 0.0f, 0.0f});
            this->link_mat->texRoughness = Texture::fromSingleColor({0.5f, 0.0f, 0.0f});
        }
        if (!joint_mat) {
            this->joint_mat = std::make_shared<PBRMaterial>();
            this->joint_mat->texAlbedo = Texture::fromSingleColor(colors::WhiteSmoke);
            this->joint_mat->texAO = Texture::fromSingleColor({1.0f, 0.0f, 0.0f});
            this->joint_mat->texMetallic = Texture::fromSingleColor({0.8f, 0.0f, 0.0f});
            this->joint_mat->texRoughness = Texture::fromSingleColor({0.8f, 0.0f, 0.0f});
        }
    }

    void render(PBRenderer& renderer, const real* q) {
        int num_joints = art->get_num_joints();
        std::vector<glmx::ttransform<real>> T_link_global(num_joints), T_joint_global(num_joints);
        calc_transforms(*art, q, T_link_global.data(), T_joint_global.data());
        for (int i = 0; i < num_joints; i++) {
            glm::mat4 link_trans = glmx::mat4_cast(T_link_global[i]);
            glm::mat4 joint_trans = glmx::mat4_cast(T_link_global[i]);
            renderer.queueRender(PBRCommand {link_meshes[i], link_mat, link_trans});
            renderer.queueRender(PBRCommand {joint_meshes[i], joint_mat, joint_trans});
        }
    }

private:
    artsim::ArticulatedBody* art;
    Ref<PBRMaterial> link_mat;
    Ref<PBRMaterial> joint_mat;
    std::vector<Ref<Mesh>> link_meshes;
    std::vector<Ref<Mesh>> joint_meshes;

};

class ArticulationStateRender {
public:
    ArticulationStateRender() = default;
    ArticulationStateRender(artsim::ArticulationState* state,
                            Ref<PBRMaterial> link_mat = {}, Ref<PBRMaterial> joint_mat = {})

    : state(state), link_mat(link_mat), joint_mat(joint_mat) {

        link_meshes.resize(state->num_joints);
        joint_meshes.resize(state->num_joints);

        for (int i = 0; i < state->num_joints; i++) {
            artsim::CollisionShape shape = state->art->links[i].col_shape;
            switch(shape.type) {
                case artsim::CollisionShape::Type::Sphere: {
                    link_meshes[i] = Mesh::makeSphere(shape.sphere.radius);
                } break;
                case artsim::CollisionShape::Type::Box: {
                    link_meshes[i] = Mesh::makeCube(shape.box.size);
                } break;
                default: {}
            }
            joint_meshes[i] = Mesh::makeSphere(0.02f);
        }

        if (!link_mat) {
            this->link_mat = std::make_shared<PBRMaterial>();
            this->link_mat->texAlbedo = Texture::fromSingleColor({0.5f, 0.0f, 0.0f});
            this->link_mat->texAO = Texture::fromSingleColor({1.0f, 0.0f, 0.0f});
            this->link_mat->texMetallic = Texture::fromSingleColor({0.5f, 0.0f, 0.0f});
            this->link_mat->texRoughness = Texture::fromSingleColor({0.5f, 0.0f, 0.0f});
        }
        if (!joint_mat) {
            this->joint_mat = std::make_shared<PBRMaterial>();
            this->joint_mat->texAlbedo = Texture::fromSingleColor(colors::WhiteSmoke);
            this->joint_mat->texAO = Texture::fromSingleColor({1.0f, 0.0f, 0.0f});
            this->joint_mat->texMetallic = Texture::fromSingleColor({0.8f, 0.0f, 0.0f});
            this->joint_mat->texRoughness = Texture::fromSingleColor({0.8f, 0.0f, 0.0f});
        }
    }

    void render(PBRenderer& renderer, DebugRenderer& debug) {
        for (int i = 0; i < state->num_joints; i++) {
            glm::mat4 link_trans = glmx::mat4_cast(state->T_link_global[i]);
            glm::mat4 joint_trans = glmx::mat4_cast(state->T_link_global[i]);
            renderer.queueRender(PBRCommand {link_meshes[i], link_mat, link_trans});
            renderer.queueRender(PBRCommand {joint_meshes[i], joint_mat, joint_trans});
        }
        for (int c = 0; c < state->contact_points.size(); c++) {
            using namespace artsim;
            auto normal = state->contact_normals[c];
            const ContactPoint& cp = state->contact_points[c];

            auto tangent_u = glmx::Ez<real>();
            auto tangent_v = glm::cross(cp.normal, tangent_u);
            auto contact_T = glmx::ttransform<real>(cp.pos, glm::tmat3x3<real>(tangent_u, tangent_v, cp.normal));

            debug.drawLine(glm::vec3(contact_T.v),
                           glm::vec3(contact_T.v + real(1) * (contact_T.R * normal)),
                           colors::Green, true);
        }
    }

private:
    artsim::ArticulationState* state = nullptr;
    Ref<PBRMaterial> link_mat;
    Ref<PBRMaterial> joint_mat;
    std::vector<Ref<Mesh>> link_meshes;
    std::vector<Ref<Mesh>> joint_meshes;

};

#endif //ARTSIM_ARTICULATION_RENDER_H
