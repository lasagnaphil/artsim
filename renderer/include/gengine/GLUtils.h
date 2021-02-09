//
// Created by lasagnaphil on 2017-03-24.
//

#pragma once

#include "glad/glad.h"
#include <vector>

class GLUtils {
public:
    static GLuint compileShader(GLenum type, const GLchar* source);
    /*
    static std::vector<Vertex> createBox(float sx, float sy, float sz);

    static std::vector<Vertex> cubeVertices;
    static std::vector<Vertex> cubeVerticesI;
    static std::vector<GLuint> cubeIndices;
     */
};

