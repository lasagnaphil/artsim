//
// Created by lasagnaphil on 8/21/21.
//

#ifndef ARTSIM_RIGID_BODY_RENDER_H
#define ARTSIM_RIGID_BODY_RENDER_H

#include <artsim/world.h>
#include <artsim/rigid_body.h>

namespace artsim {

class RigidBodyRender {
public:
    RigidBodyRender() = default;
    RigidBodyRender(World* world, Id<RigidBody> rb_id,
                    Ref<PBRMaterial> mat)
                    : world(world), rb_id(rb_id), mat(mat) {

        auto& rb = *world->get_rigid_body(rb_id);
        artsim::CollisionShape shape = rb.spec.col_shape;
        switch(shape.type) {
            case artsim::CollisionShape::Type::Sphere: {
                mesh = Mesh::makeSphere();
            } break;
            case artsim::CollisionShape::Type::Box: {
                mesh = Mesh::makeCube();
            } break;
            case artsim::CollisionShape::Type::Mesh: {
                auto col_mesh = world->get_collision_mesh(shape.mesh.id)->mesh.get();
                mesh = Resources::make<Mesh>();
                auto& render_mesh = *mesh;
                render_mesh.vertices.resize(3 * col_mesh->nFaces());
                for (int k = 0; k < 3 * col_mesh->nFaces(); k++) {
                    render_mesh.vertices[k].normal = glm::rvec3(0);
                }
                for (int k = 0; k < col_mesh->nFaces(); k++) {
                    auto& face = col_mesh->face(k);
                    glm::rvec3 normal = to_glm_vec(col_mesh->computeFaceNormal(k));
                    render_mesh.vertices[3*k].pos = to_glm_vec(col_mesh->vertex(face[0]));
                    render_mesh.vertices[3*k].normal += normal;
                    render_mesh.vertices[3*k].uv = glm::rvec2(0);
                    render_mesh.vertices[3*k+1].pos = to_glm_vec(col_mesh->vertex(face[1]));
                    render_mesh.vertices[3*k+1].normal += normal;
                    render_mesh.vertices[3*k+1].uv = glm::rvec2(0);
                    render_mesh.vertices[3*k+2].pos = to_glm_vec(col_mesh->vertex(face[2]));
                    render_mesh.vertices[3*k+2].normal += normal;
                    render_mesh.vertices[3*k+2].uv = glm::rvec2(0);
                }
                for (int k = 0; k < 3 * col_mesh->nFaces(); k++) {
                    render_mesh.vertices[k].normal = glm::normalize(render_mesh.vertices[k].normal);
                }
                render_mesh.initVBO();
            } break;
            default: {}
        }

        if (!this->mat) {
            this->mat = std::make_shared<PBRMaterial>();
            this->mat->texAlbedo = Texture::fromSingleColor(colors::WhiteSmoke);
            this->mat->texAO = Texture::fromSingleColor({1.0f, 0.0f, 0.0f});
            this->mat->texMetallic = Texture::fromSingleColor({0.8f, 0.0f, 0.0f});
            this->mat->texRoughness = Texture::fromSingleColor({0.8f, 0.0f, 0.0f});
        }
    }

    void render(PBRenderer& renderer) {
        auto& rb = *world->get_rigid_body(rb_id);
        glm::mat4 trans = glmx::mat4_cast(rquat_transform(rb.pos, rb.rot));
        trans = glm::scale(trans, glm::vec3(rb.spec.col_shape.scale));
        renderer.queueRender(PBRCommand {mesh, mat, trans});
    }

    World* world;
    Id<RigidBody> rb_id;
    Ref<PBRMaterial> mat;
    Ref<Mesh> mesh;
};

}
#endif //ARTSIM_RIGID_BODY_RENDER_H
