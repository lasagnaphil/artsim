//
// Created by lasagnaphil on 2017-03-28.
//

#include <iostream>
#include "gengine/GLUtils.h"

GLuint GLUtils::compileShader(GLenum type, const GLchar *source) {
    GLuint shader = glCreateShader(type);

    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);
    GLint status;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);

    if (status == GL_TRUE) {
        std::cout << "Shader (type " << type << ") is compiled successfully!" << std::endl;
    } else {
        std::cout << "Shader (type " << type << ") compile failed!" << std::endl;
    }

    std::cout << "Compile log: " << std::endl;
    char compileInfo[512];
    glGetShaderInfoLog(shader, 512, NULL, compileInfo);
    std::cout << compileInfo << std::endl;

    return shader;
}

/*
std::vector<Vertex> GLUtils::createBox(float sx, float sy, float sz) {
    std::vector<Vertex> box = std::vector<Vertex>(cubeVertices.begin(), cubeVertices.end());
    for (auto& vertex : box) {
        vertex.pos.x *= sx;
        vertex.pos.y *= sy;
        vertex.pos.z *= sz;
    }
    return box;
}

std::vector<Vertex> GLUtils::cubeVertices = {
    Vertex(-0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f),
    Vertex(0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f),
    Vertex(0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f),
    Vertex(0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f),
    Vertex(-0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f),
    Vertex(-0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f),
    Vertex(-0.5f, -0.5f,  0.5f,  0.0f,  0.0f, 1.0f),
    Vertex(0.5f, -0.5f,  0.5f,  0.0f,  0.0f, 1.0f),
    Vertex(0.5f,  0.5f,  0.5f,  0.0f,  0.0f, 1.0f),
    Vertex(0.5f,  0.5f,  0.5f,  0.0f,  0.0f, 1.0f),
    Vertex(-0.5f,  0.5f,  0.5f,  0.0f,  0.0f, 1.0f),
    Vertex(-0.5f, -0.5f,  0.5f,  0.0f,  0.0f, 1.0f),
    Vertex(-0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f),
    Vertex(-0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f),
    Vertex(-0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f),
    Vertex(-0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f),
    Vertex(-0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f),
    Vertex(-0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f),
    Vertex(0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f),
    Vertex(0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f),
    Vertex(0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f),
    Vertex(0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f),
    Vertex(0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f),
    Vertex(0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f),
    Vertex(-0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f),
    Vertex(0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f),
    Vertex(0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f),
    Vertex(0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f),
    Vertex(-0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f),
    Vertex(-0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f),
    Vertex(-0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f),
    Vertex(0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f),
    Vertex(0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f),
    Vertex(0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f),
    Vertex(-0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f),
    Vertex(-0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f)
};
std::vector<Vertex> GLUtils::cubeVerticesI = {
        Vertex(-0.5f, -0.5f, 0.5f),
        Vertex(0.5f, -0.5f, 0.5f),
        Vertex(0.5f, 0.5f, 0.5f),
        Vertex(-0.5f, 0.5f, 0.5f),
        Vertex(-0.5f, -0.5f, -0.5f),
        Vertex(0.5f, -0.5f, -0.5f),
        Vertex(0.5f, 0.5f, -0.5f),
        Vertex(-0.5f, 0.5f, -0.5f)
};
std::vector<GLuint> GLUtils::cubeIndices = {
        // front
        0, 1, 2,
        2, 3, 0,
        // top
        1, 5, 6,
        6, 2, 1,
        // back
        7, 6, 5,
        5, 4, 7,
        // bottom
        4, 0, 3,
        3, 7, 4,
        // left
        4, 5, 1,
        1, 0, 4,
        // right
        3, 2, 6,
        6, 7, 3,
};

 */
