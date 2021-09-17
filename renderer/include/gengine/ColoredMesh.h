//
// Created by lasagnaphil on 21. 9. 8..
//

#ifndef EOS_SCAN_TO_HUMAN_COLOREDMESH_H
#define EOS_SCAN_TO_HUMAN_COLOREDMESH_H

#include "gengine/Shader.h"
#include "gengine/Arena.h"

struct ColoredMesh {
    struct Vertex {
        glm::vec3 pos;
        glm::vec3 color;
    };
    enum class DrawMode {
        Static, Dynamic
    };

    std::vector<Vertex> vertices;

    GLuint vao, vbo;
    GLenum drawType = GL_TRIANGLES;

    ColoredMesh(std::vector<Vertex> vertices = {}) : vertices(std::move(vertices)) {}

    void initVBO(DrawMode drawMode = DrawMode::Static);
    void updateVBO();
};
#endif //EOS_SCAN_TO_HUMAN_COLOREDMESH_H
