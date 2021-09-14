//
// Created by lasagnaphil on 19. 3. 16.
//

#include "gengine/Mesh.h"
#include <unordered_map>
#include <algorithm>
#include <mutex>

#include "tiny_obj_loader.h"
#include <glm/gtx/norm.hpp>

static std::mutex g_mutex;

float Mesh::cubeVertices[8*36] = {
        -0.5f, -0.5f, -0.5f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f,
        0.5f, -0.5f, -0.5f, 0.0f, 0.0f, -1.0f, 1.0f, 0.0f,
        0.5f, 0.5f, -0.5f, 0.0f, 0.0f, -1.0f, 1.0f, 1.0f,
        0.5f, 0.5f, -0.5f, 0.0f, 0.0f, -1.0f, 1.0f, 1.0f,
        -0.5f, 0.5f, -0.5f, 0.0f, 0.0f, -1.0f, 0.0f, 1.0f,
        -0.5f, -0.5f, -0.5f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f,

        -0.5f, -0.5f, 0.5f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
        0.5f, -0.5f, 0.5f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f,
        0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f,
        0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f,
        -0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f,
        -0.5f, -0.5f, 0.5f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,

        -0.5f, 0.5f, 0.5f, -1.0f, 0.0f, 0.0f, 1.0f, 0.0f,
        -0.5f, 0.5f, -0.5f, -1.0f, 0.0f, 0.0f, 1.0f, 1.0f,
        -0.5f, -0.5f, -0.5f, -1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
        -0.5f, -0.5f, -0.5f, -1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
        -0.5f, -0.5f, 0.5f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        -0.5f, 0.5f, 0.5f, -1.0f, 0.0f, 0.0f, 1.0f, 0.0f,

        0.5f, 0.5f, 0.5f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f,
        0.5f, 0.5f, -0.5f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f,
        0.5f, -0.5f, -0.5f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
        0.5f, -0.5f, -0.5f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
        0.5f, -0.5f, 0.5f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.5f, 0.5f, 0.5f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f,

        -0.5f, -0.5f, -0.5f, 0.0f, -1.0f, 0.0f, 0.0f, 1.0f,
        0.5f, -0.5f, -0.5f, 0.0f, -1.0f, 0.0f, 1.0f, 1.0f,
        0.5f, -0.5f, 0.5f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f,
        0.5f, -0.5f, 0.5f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f,
        -0.5f, -0.5f, 0.5f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f,
        -0.5f, -0.5f, -0.5f, 0.0f, -1.0f, 0.0f, 0.0f, 1.0f,

        -0.5f, 0.5f, -0.5f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f,
        0.5f, 0.5f, -0.5f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f,
        0.5f, 0.5f, 0.5f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f,
        0.5f, 0.5f, 0.5f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f,
        -0.5f, 0.5f, 0.5f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
        -0.5f, 0.5f, -0.5f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f
};

float Mesh::planeVertices[8*6] = {
        -0.5f, 0.0f, -0.5f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
        0.5f, 0.0f, -0.5f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f,
        0.5f, 0.0f, 0.5f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f,

        -0.5f, 0.0f, -0.5f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
        0.5f, 0.0f, 0.5f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f,
        -0.5f, 0.0f, 0.5f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f,
};

void Mesh::initVBO(DrawMode drawMode) {
    int32_t drawModeGL = drawMode == DrawMode::Static? GL_STATIC_DRAW : GL_DYNAMIC_DRAW;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Mesh::Vertex) * vertices.size(), vertices.data(), drawModeGL);

    if (!indices.empty()) {
        glGenBuffers(1, &ebo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint32_t) * indices.size(), indices.data(), drawModeGL);
    }

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)offsetof(Mesh::Vertex, normal));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)offsetof(Mesh::Vertex, uv));
    glBindVertexArray(0);
}

void Mesh::updateVBO() {
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Mesh::Vertex) * vertices.size(), vertices.data());
}


MeshCollider Mesh::generateCollider() {
    assert (indices.size() == 0 && "Meshes with index buffer not supported");

    MeshCollider collider;

    // TODO: This is a very inefficient O(n^2) algorithm.
    const float threshold = 1e-6;
    for (auto& vertex : vertices) {
        int i;
        for (i = 0; i < collider.points.size(); i++) {
            glm::vec3 p = collider.points[i];
            if (glm::length2(p - vertex.pos) < threshold) {
                collider.indices.push_back(i);
                break;
            }
        }
        if (i == collider.points.size()) {
            collider.indices.push_back(collider.points.size());
            collider.points.push_back(vertex.pos);
        }
    }

    return collider;
}

void Mesh::rotate(glm::quat rot) {
    for (auto& vertex : vertices) {
        vertex.pos = rot * vertex.pos;
        vertex.normal = rot * vertex.normal;
    }
}

void Mesh::sortVertices(glmx::transform meshTrans, glm::vec3 viewDir) {
    if (!indices.empty()) {
        fprintf(stderr, "sortVertices() not supported for mesh with index buffer\n");
        exit(EXIT_FAILURE);
    }
    struct SortVertex {
        int i;
        float z;
    };

    std::vector<SortVertex> sortedVertices(vertices.size()/3);
    for (int i = 0; i < vertices.size()/3; i++) {
        auto& v = sortedVertices[i];
        v.i = i;
        auto vpos1 = vertices[3*i+0].pos;
        auto vpos2 = vertices[3*i+1].pos;
        auto vpos3 = vertices[3*i+2].pos;
        v.z = glm::dot(viewDir, meshTrans.R * (vpos1 + vpos2 + vpos3));
    }
    std::sort(sortedVertices.begin(), sortedVertices.end(), [](const SortVertex& a, const SortVertex& b) {
        return a.z < b.z;
    });

    auto tempVertices = vertices;
    for (int i = 0; i < vertices.size()/3; i++) {
        int idx = sortedVertices[i].i;
        vertices[3*i+0] = tempVertices[3*idx+0];
        vertices[3*i+1] = tempVertices[3*idx+1];
        vertices[3*i+2] = tempVertices[3*idx+2];
    }
}

void Mesh::updateOBJInternal(const glm::vec3* vertices, const glm::ivec3* triangles, int num_triangles, OUT glm::vec3* normals) {
    for (int t = 0; t < num_triangles; t++) {
        this->vertices[3*t+0].pos = vertices[triangles[t][0]];
        this->vertices[3*t+0].normal = glm::vec3(0);
        this->vertices[3*t+0].uv = glm::vec2(0);
        this->vertices[3*t+1].pos = vertices[triangles[t][1]];
        this->vertices[3*t+1].normal = glm::vec3(0);
        this->vertices[3*t+1].uv = glm::vec2(0);
        this->vertices[3*t+2].pos = vertices[triangles[t][2]];
        this->vertices[3*t+2].normal = glm::vec3(0);
        this->vertices[3*t+2].uv = glm::vec2(0);
        normals[triangles[t][0]] = glm::vec3(0);
        normals[triangles[t][1]] = glm::vec3(0);
        normals[triangles[t][2]] = glm::vec3(0);
    }
    for (int t = 0; t < num_triangles; t++) {
        auto v0 = this->vertices[3*t+0].pos;
        auto v1 = this->vertices[3*t+1].pos;
        auto v2 = this->vertices[3*t+2].pos;
        auto n = glm::normalize(glm::cross(v1 - v0, v2 - v0));
        normals[triangles[t][0]] += n;
        normals[triangles[t][1]] += n;
        normals[triangles[t][2]] += n;
    }
    // TODO: Remove repetitive normalization
    for (int t = 0; t < num_triangles; t++) {
        normals[triangles[t][0]] = glm::normalize(normals[triangles[t][0]]);
        normals[triangles[t][1]] = glm::normalize(normals[triangles[t][1]]);
        normals[triangles[t][2]] = glm::normalize(normals[triangles[t][2]]);
        this->vertices[3*t+0].normal = normals[triangles[t][0]];
        this->vertices[3*t+1].normal = normals[triangles[t][0]];
        this->vertices[3*t+2].normal = normals[triangles[t][0]];
    }
}

void Mesh::updateOBJ(const glm::vec3* vertices, const glm::ivec3* triangles, int num_triangles, OUT glm::vec3* normals) {
    updateOBJInternal(vertices, triangles, num_triangles, normals);
    updateVBO();
}

Ref<Mesh> Mesh::fromOBJ(const artsim::OBJFile* objfile, DrawMode mode) {
    int num_tris = objfile->triangle_vertices.size();
    std::vector<Vertex> vertices(3*num_tris);
    for (int i = 0; i < num_tris; i++) {
        auto tri_pos = objfile->triangle_vertices[i];
        auto tri_norm = objfile->triangle_normals[i];
        auto tri_uv = objfile->triangle_uvs[i];
        vertices[3*i+0].pos = objfile->vertices[tri_pos[0]];
        vertices[3*i+0].normal = objfile->normals[tri_norm[0]];
        vertices[3*i+0].uv = objfile->uvs[tri_uv[0]];
        vertices[3*i+1].pos = objfile->vertices[tri_pos[1]];
        vertices[3*i+1].normal = objfile->normals[tri_norm[1]];
        vertices[3*i+1].uv = objfile->uvs[tri_uv[1]];
        vertices[3*i+2].pos = objfile->vertices[tri_pos[2]];
        vertices[3*i+2].normal = objfile->normals[tri_norm[2]];
        vertices[3*i+2].uv = objfile->uvs[tri_uv[2]];
    }
    auto mesh = Resources::make<Mesh>(vertices);
    mesh->initVBO(mode);
    return mesh;
}

Ref<Mesh> Mesh::fromOBJ(const char* filename, DrawMode mode) {
    using namespace tinyobj;

    ObjReader reader;
    reader.ParseFromFile(filename);
    if (reader.Valid()) {
        return Mesh::fromOBJ(reader.GetAttrib(), reader.GetShapes().data(), reader.GetShapes().size(), mode);
    }
    else {
        fprintf(stderr, "Error in Mesh::fromOBJ: failed to parse OBJ file %s\n", filename);
        fprintf(stderr, "Message: %s\n", reader.Error().c_str());
        return {};
    }
}

Ref<Mesh> Mesh::fromOBJ(const tinyobj::attrib_t& attrib, const tinyobj::shape_t* shapes, int num_shapes, DrawMode mode) {
    Ref<Mesh> mesh = Resources::make<Mesh>();
    for (int sidx = 0; sidx < num_shapes; sidx++) {
        auto& shape = shapes[sidx];
        for (const auto& index : shape.mesh.indices) {
            Mesh::Vertex vertex;
            vertex.pos = {
                    attrib.vertices[3 * index.vertex_index + 0],
                    attrib.vertices[3 * index.vertex_index + 1],
                    attrib.vertices[3 * index.vertex_index + 2]
            };
            vertex.normal = {
                    attrib.normals[3 * index.normal_index + 0],
                    attrib.normals[3 * index.normal_index + 1],
                    attrib.normals[3 * index.normal_index + 2]
            };
            if (!attrib.texcoords.empty()) {
                vertex.uv = {
                        attrib.texcoords[2 * index.texcoord_index + 0],
                        attrib.texcoords[2 * index.texcoord_index + 1]
                };
            }
            mesh->vertices.push_back(vertex);
        }
    }

    mesh->initVBO(mode);
    return mesh;
}

Ref<Mesh> Mesh::fromOBJ(const glm::vec3* vertices, const glm::ivec3* triangles, int num_triangles, OUT glm::vec3* normals, DrawMode mode) {
    Ref<Mesh> mesh = Resources::make<Mesh>();
    mesh->vertices.resize(3*num_triangles);
    mesh->updateOBJInternal(vertices, triangles, num_triangles, normals);
    mesh->initVBO(mode);
    return mesh;
}


Ref<Mesh> Mesh::makeCube(const glm::vec3 &scale) {
    std::vector<Vertex> vertices((Vertex*)cubeVertices, ((Vertex*)cubeVertices) + 36);
    for (int i = 0; i < 36; i++) {
        vertices[i].pos *= scale;
    }
    auto mesh = Resources::make<Mesh>(vertices);
    mesh->initVBO();
    return mesh;
}

Ref<Mesh> Mesh::makePlane(float size, float uvSize) {
    std::vector<Vertex> vertices((Vertex*)planeVertices, ((Vertex*)planeVertices) + 6);
    for (int i = 0; i < 6; i++) {
        vertices[i].pos *= size;
        vertices[i].uv *= uvSize;
    }
    auto mesh = Resources::make<Mesh>(vertices);
    mesh->initVBO();
    return mesh;
}

Ref<Mesh> Mesh::makeCylinder(unsigned int numQuads, float r, float h) {
    std::vector<Vertex> vertices;
    vertices.reserve(numQuads * 6);
    float delta = glm::two_pi<float>() / numQuads;
    for (unsigned int i = 0; i < numQuads; ++i) {
        float theta = i * delta;
        float thetap = (i + 1) * delta;
        glm::vec3 normal(r * glm::cos(theta), h, -r * glm::sin(theta));
        glm::vec3 normalp(r * glm::cos(thetap), h, -r * glm::sin(thetap));
        vertices.push_back(Vertex{{r * glm::cos(theta), 0, -r * glm::sin(theta)}, normal, {0.0f, 0.0f}});
        vertices.push_back(Vertex{{r * glm::cos(theta), h, -r * glm::sin(theta)}, normal, {0.0f, 1.0f}});
        vertices.push_back(Vertex{{r * glm::cos(thetap), h, -r * glm::sin(thetap)}, normalp, {1.0f, 1.0f}});
        vertices.push_back(Vertex{{r * glm::cos(theta), 0, -r * glm::sin(theta)}, normal, {0.0f, 0.0f}});
        vertices.push_back(Vertex{{r * glm::cos(thetap), h, -r * glm::sin(thetap)}, normalp, {1.0f, 1.0f}});
        vertices.push_back(Vertex{{r * glm::cos(thetap), 0, -r * glm::sin(thetap)}, normalp, {1.0f, 0.0f}});
    }
    auto mesh = Resources::make<Mesh>(vertices);
    mesh->initVBO();
    return mesh;
}

Ref<Mesh> Mesh::makeCone(unsigned int numTriangles, float r, float h) {
    std::vector<Vertex> vertices;
    vertices.reserve(numTriangles * 3);
    float delta = glm::two_pi<float>() / numTriangles;
    for (unsigned int i = 0; i < numTriangles; ++i) {
        float theta = i * delta;
        float thetap = (i + 1) * delta;
        float thetam = (theta + thetap) / 2;
        float slopeLen = glm::sqrt(r*r + h*h);
        glm::vec3 normal(glm::cos(theta) * h / slopeLen, r / slopeLen, -glm::sin(theta) * h / slopeLen);
        glm::vec3 normalm(glm::cos(thetam) * h / slopeLen, r / slopeLen, -glm::sin(thetam) * h / slopeLen);
        glm::vec3 normalp(glm::cos(thetap) * h / slopeLen, r / slopeLen, -glm::sin(thetap) * h / slopeLen);
        vertices.push_back(Vertex{{0.0f, h, 0.0f}, normalm, {0.0f, 0.0f}});
        vertices.push_back(Vertex{{r * glm::cos(theta), 0, -r * glm::sin(theta)}, normal, {1.0f, 0.0f}});
        vertices.push_back(Vertex{{r * glm::cos(thetap), 0, -r * glm::sin(thetap)}, normalp, {0.0f, 1.0f}});
    }
    auto mesh = Resources::make<Mesh>(vertices);
    mesh->initVBO();
    return mesh;
}

// Courtesy of http://www.songho.ca/opengl/gl_sphere.html
Ref<Mesh> Mesh::makeSphere(float radius, unsigned int sectorCount, unsigned int stackCount) {
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;

    vertices.reserve((stackCount + 1) * (sectorCount + 1));
    indices.reserve(3 * (stackCount + 1) * (sectorCount + 1));

    float x, y, z, xy;                              // vertex position
    float nx, ny, nz, lengthInv = 1.0f / radius;    // vertex normal
    float s, t;                                     // vertex texCoord

    float sectorStep = 2 * M_PI / sectorCount;
    float stackStep = M_PI / stackCount;
    float sectorAngle, stackAngle;

    for(int i = 0; i <= stackCount; ++i)
    {
        stackAngle = M_PI / 2 - i * stackStep;        // starting from pi/2 to -pi/2
        xy = radius * cosf(stackAngle);             // r * cos(u)
        z = radius * sinf(stackAngle);              // r * sin(u)

        // add (sectorCount+1) vertices per stack
        // the first and last vertices have same position and normal, but different tex coords
        for(int j = 0; j <= sectorCount; ++j)
        {
            sectorAngle = j * sectorStep;           // starting from 0 to 2pi

            // vertex position (x, y, z)
            x = xy * cosf(sectorAngle);             // r * cos(u) * cos(v)
            y = xy * sinf(sectorAngle);             // r * cos(u) * sin(v)

            // normalized vertex normal (nx, ny, nz)
            nx = x * lengthInv;
            ny = y * lengthInv;
            nz = z * lengthInv;

            // vertex tex coord (s, t) range between [0, 1]
            s = (float)j / sectorCount;
            t = (float)i / stackCount;

            vertices.push_back(Vertex {{x, y, z}, {nx, ny, nz}, {s, t}});
        }
    }

    int k1, k2;
    for(int i = 0; i < stackCount; ++i)
    {
        k1 = i * (sectorCount + 1);     // beginning of current stack
        k2 = k1 + sectorCount + 1;      // beginning of next stack

        for(int j = 0; j < sectorCount; ++j, ++k1, ++k2)
        {
            // 2 triangles per sector excluding first and last stacks
            // k1 => k2 => k1+1
            if(i != 0)
            {
                indices.push_back(k1);
                indices.push_back(k2);
                indices.push_back(k1 + 1);
            }

            // k1+1 => k2 => k2+1
            if(i != (stackCount-1))
            {
                indices.push_back(k1 + 1);
                indices.push_back(k2);
                indices.push_back(k2 + 1);
            }
        }
    }

    auto mesh = Resources::make<Mesh>(vertices, indices);
    mesh->initVBO();
    return mesh;
}

Ref<Mesh> Mesh::makeCapsule(float radius, float height, unsigned int sectorCount, unsigned int stackCount) {
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;

    vertices.reserve((stackCount + 2) * (sectorCount + 1));
    indices.reserve(3 * (stackCount + 2) * (sectorCount + 1));

    float x, y, z, xy;                              // vertex position
    float nx, ny, nz, lengthInv = 1.0f / radius;    // vertex normal
    float s, t;                                     // vertex texCoord

    float sectorStep = 2 * M_PI / sectorCount;
    float stackStep = M_PI / stackCount;
    float sectorAngle, stackAngle;

    // northern hemisphere
    for(int i = 0; i <= stackCount/2; ++i)
    {
        stackAngle = M_PI / 2 - i * stackStep;        // starting from pi/2 to -pi/2
        xy = radius * cosf(stackAngle);             // r * cos(u)
        z = height / 2 + radius * sinf(stackAngle);              // r * sin(u)

        // add (sectorCount+1) vertices per stack
        // the first and last vertices have same position and normal, but different tex coords
        for(int j = 0; j <= sectorCount; ++j)
        {
            sectorAngle = j * sectorStep;           // starting from 0 to 2pi

            // vertex position (x, y, z)
            x = xy * cosf(sectorAngle);             // r * cos(u) * cos(v)
            y = xy * sinf(sectorAngle);             // r * cos(u) * sin(v)

            // normalized vertex normal (nx, ny, nz)
            nx = x * lengthInv;
            ny = y * lengthInv;
            nz = z * lengthInv;

            // vertex tex coord (s, t) range between [0, 1]
            s = (float)j / sectorCount;
            t = (float)i / stackCount;

            vertices.push_back(Vertex {{x, y, z}, {nx, ny, nz}, {s, t}});
        }
    }
    // southern hemisphere
    for(int i = stackCount/2; i <= stackCount; ++i)
    {
        stackAngle = M_PI / 2 - i * stackStep;        // starting from pi/2 to -pi/2
        xy = radius * cosf(stackAngle);             // r * cos(u)
        z = -height / 2 + radius * sinf(stackAngle);              // r * sin(u)

        // add (sectorCount+1) vertices per stack
        // the first and last vertices have same position and normal, but different tex coords
        for(int j = 0; j <= sectorCount; ++j)
        {
            sectorAngle = j * sectorStep;           // starting from 0 to 2pi

            // vertex position (x, y, z)
            x = xy * cosf(sectorAngle);             // r * cos(u) * cos(v)
            y = xy * sinf(sectorAngle);             // r * cos(u) * sin(v)

            // normalized vertex normal (nx, ny, nz)
            nx = x * lengthInv;
            ny = y * lengthInv;
            nz = z * lengthInv;

            // vertex tex coord (s, t) range between [0, 1]
            s = (float)j / sectorCount;
            t = (float)i / stackCount;

            vertices.push_back(Vertex {{x, y, z}, {nx, ny, nz}, {s, t}});
        }
    }

    int k1, k2;
    for(int i = 0; i < stackCount; ++i)
    {
        k1 = i * (sectorCount + 1);     // beginning of current stack
        k2 = k1 + sectorCount + 1;      // beginning of next stack

        for(int j = 0; j < sectorCount; ++j, ++k1, ++k2)
        {
            // 2 triangles per sector excluding first and last stacks
            // k1 => k2 => k1+1
            if(i != 0)
            {
                indices.push_back(k1);
                indices.push_back(k2);
                indices.push_back(k1 + 1);
            }

            // k1+1 => k2 => k2+1
            if(i != (stackCount-1))
            {
                indices.push_back(k1 + 1);
                indices.push_back(k2);
                indices.push_back(k2 + 1);
            }
        }
    }

    auto mesh = Resources::make<Mesh>(vertices, indices);
    mesh->initVBO();
    return mesh;
}
