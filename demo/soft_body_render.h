//
// Created by lasagnaphil on 2/12/21.
//

#ifndef ARTSIM_SOFT_BODY_RENDER_H
#define ARTSIM_SOFT_BODY_RENDER_H

#include <artsim/soft_body.h>

class SoftBodyRender {
public:
    SoftBodyRender() = default;

    SoftBodyRender(artsim::SoftBodyData* data, Ref<PBRMaterial> mat) : data(data), mat(mat) {
        std::vector<Mesh::Vertex> vertices(3*data->triangles.size());

        mesh = Resources::make<Mesh>(vertices);
        mesh->initVBO(Mesh::DrawMode::Dynamic);
        update_mesh(data->vertices.data());
    }

    void render(PBRenderer& renderer, DebugRenderer& debug, const glm::tvec3<artsim::real>* vpos) {
        update_mesh(vpos);

        renderer.queueRender({mesh, mat, glm::mat4(1.0f)});

        for (int t = 0; t < data->triangles.size(); t++) {
            auto i0 = data->triangles[t][0];
            auto i1 = data->triangles[t][1];
            auto i2 = data->triangles[t][2];
            debug.drawLine(vpos[i0], vpos[i1], colors::Black, true);
            debug.drawLine(vpos[i0], vpos[i2], colors::Black, true);
            debug.drawLine(vpos[i1], vpos[i2], colors::Black, true);
        }
    }

    void update_mesh(const glm::tvec3<artsim::real>* vpos) {
        Mesh& m = *mesh;
        for (int t = 0; t < data->triangles.size(); t++) {
            m.vertices[3*t+0].pos = vpos[data->triangles[t][0]];
            m.vertices[3*t+0].normal = glm::vec3(0);
            m.vertices[3*t+0].uv = glm::vec2(0);
            m.vertices[3*t+1].pos = vpos[data->triangles[t][1]];
            m.vertices[3*t+1].normal = glm::vec3(0);
            m.vertices[3*t+1].uv = glm::vec2(0);
            m.vertices[3*t+2].pos = vpos[data->triangles[t][2]];
            m.vertices[3*t+2].normal = glm::vec3(0);
            m.vertices[3*t+2].uv = glm::vec2(0);
        }

        for (int t = 0; t < data->triangles.size(); t++) {
            auto v0 = m.vertices[3*t+0].pos;
            auto v1 = m.vertices[3*t+1].pos;
            auto v2 = m.vertices[3*t+2].pos;
            auto n = glm::normalize(glm::cross(v1 - v0, v2 - v0));
            m.vertices[3*t+0].normal += n;
            m.vertices[3*t+1].normal += n;
            m.vertices[3*t+2].normal += n;
        }
        for (int i = 0; i < m.vertices.size(); i++) {
            m.vertices[i].normal = glm::normalize(m.vertices[i].normal);
        }

        m.updateVBO();
    }

private:
    artsim::SoftBodyData* data;
    Ref<Mesh> mesh;
    Ref<PBRMaterial> mat;
};

#endif //ARTSIM_SOFT_BODY_RENDER_H
