//
// Created by lasagnaphil on 19. 3. 8.
//

#ifndef MOTION_EDITING_UVMESH_H
#define MOTION_EDITING_UVMESH_H

#include <glm/vec3.hpp>
#include <glm/vec2.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/trigonometric.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glad/glad.h>
#include <artsim/math/se3.h>
#include <artsim/obj_file.h>

#include <vector>

#include "gengine/Shader.h"
#include "gengine/Arena.h"

#include "tiny_obj_loader.h"

struct MeshCollider {
    std::vector<glm::vec3> points;
    std::vector<uint32_t> indices;
};

struct Mesh {
    static float cubeVertices[8*36];
    static float planeVertices[8*6];

    struct Vertex {
        glm::vec3 pos;
        glm::vec3 normal;
        glm::vec2 uv;
    };

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;

    GLuint vao;
    GLuint vbo;
    GLuint ebo;

    Mesh(std::vector<Vertex> vertices = {}, std::vector<uint32_t> indices = {}) :
        vertices(std::move(vertices)), indices(std::move(indices)) {}
    Mesh(Vertex* vertexData, std::size_t vertexCount) :
        vertices(vertexData, vertexData + vertexCount), indices() {}
    Mesh(Vertex* vertexData, std::size_t vertexCount, uint32_t* indexData, std::size_t indexCount) :
        vertices(vertexData, vertexData + vertexCount), indices(indexData, indexData + indexCount) {}

    enum class DrawMode {
        Static, Dynamic
    };
    void initVBO(DrawMode drawMode = DrawMode::Static);
    void updateVBO();

    MeshCollider generateCollider();

    void rotate(glm::quat rot);
    void sortVertices(glmx::transform meshTrans, glm::vec3 viewDir);

    static Ref<Mesh> fromOBJ(const artsim::OBJFile* objfile);
    static Ref<Mesh> fromOBJ(const char* filename);
    static Ref<Mesh> fromOBJ(const tinyobj::attrib_t& attrib, const tinyobj::shape_t* shapes, int num_shapes);
    static Ref<Mesh> makeCube(const glm::vec3& scale = {1.0f, 1.0f, 1.0f});
    static Ref<Mesh> makePlane(float size = 1.0f, float uvSize = 1.0f);
    static Ref<Mesh> makeCylinder(unsigned int numQuads, float r, float h);
    static Ref<Mesh> makeCone(unsigned int numTriangles, float r, float h);
    static Ref<Mesh> makeSphere(float radius = 1.0f, unsigned int sectorCount = 36, unsigned int stackCount = 18);
    static Ref<Mesh> makeCapsule(float radius = 1.0f, float height = 1.0f, unsigned int sectorCount = 36, unsigned int stackCount = 18);
};
#endif //MOTION_EDITING_UVMESH_H
