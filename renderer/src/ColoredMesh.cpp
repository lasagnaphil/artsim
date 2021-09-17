//
// Created by lasagnaphil on 21. 9. 8..
//

#include "gengine/ColoredMesh.h"

void ColoredMesh::initVBO(ColoredMesh::DrawMode drawMode) {
    int32_t drawModeGL = drawMode == DrawMode::Static? GL_STATIC_DRAW : GL_DYNAMIC_DRAW;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(ColoredMesh::Vertex) * vertices.size(), vertices.data(), drawModeGL);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(ColoredMesh::Vertex), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(ColoredMesh::Vertex), (void*)offsetof(ColoredMesh::Vertex, color));
    glBindVertexArray(0);
}

void ColoredMesh::updateVBO() {
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(ColoredMesh::Vertex) * vertices.size(), vertices.data());
}
