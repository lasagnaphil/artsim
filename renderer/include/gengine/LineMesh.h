//
// Created by lasagnaphil on 19. 3. 11.
//

#ifndef MOTION_EDITING_LINEMESH_H
#define MOTION_EDITING_LINEMESH_H

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glad/glad.h>
#include <span.hpp>

#include "gengine/Arena.h"

struct LineMaterial {
    GLenum lineType;
    bool drawLines = true;
    bool drawPoints = true;
    glm::vec4 lineColor;
    glm::vec4 pointColor;
    float lineWidth;
    float pointSize;
};

struct LineMesh {
    LineMesh() = default;

    LineMesh(std::vector<glm::vec3> positions) :
        positions(std::move(positions)) {}

    void init();

    void addVertex(glm::vec3 pos) {
        positions.push_back(pos);
    }

    void removeAtIndex(std::size_t i) {
        positions.erase(positions.begin() + i);
    }

    void setPositions(std::vector<glm::vec3> p) {
        positions = std::move(p);
    }

    void subdivisionBSpline(int interpFactor);

    void updateBuffers();

    void updateAllPoints(std::span<glm::vec3> points);

    void updatePoint(std::size_t idx, const glm::vec3& point);

    std::vector<glm::vec3> positions;

    GLuint vao;
    GLuint vbo;
};

#endif //MOTION_EDITING_LINEMESH_H
