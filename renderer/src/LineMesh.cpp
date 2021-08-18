//
// Created by lasagnaphil on 19. 3. 11.
//

#include "gengine/LineMesh.h"
#include "gengine/Shader.h"

void LineMesh::init() {
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * positions.size(), positions.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glBindVertexArray(0);
}

void LineMesh::subdivisionBSpline(int interpFactor) {
    std::vector<glm::vec3> prev = positions;
    std::vector<glm::vec3> next;
    int numVertices = positions.size();
    for (int iter = 0; iter < interpFactor; iter++) {
        next.resize(2*numVertices+1);
        next[0] = prev[0];
        for (int i = 0; i < numVertices; i++) {
            int iprev = (i == 0)? 0 : (i - 1);
            int inext = (i == numVertices - 1)? numVertices - 1 : (i + 1);
            next[2*i + 1] = (prev[iprev] + 6.f * prev[i] + prev[inext]) / 8.f;
            next[2*i + 2] = (prev[i] + prev[inext]) / 2.f;
        }
        numVertices = 2*numVertices + 1;
        std::swap(prev, next);
    }
    positions = prev;

}

void LineMesh::updateBuffers() {
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(glm::vec3) * positions.size(), positions.data());
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void LineMesh::updateAllPoints(std::span<glm::vec3> points) {
    assert(points.size() == positions.size());
    positions = std::vector<glm::vec3>(points.begin(), points.end());
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(glm::vec3) * points.size(), points.data());
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void LineMesh::updatePoint(std::size_t idx, const glm::vec3& point) {
    positions[idx] = point;
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferSubData(GL_ARRAY_BUFFER, sizeof(glm::vec3)*idx, sizeof(glm::vec3), &positions[idx]);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

