//
// Created by lasagnaphil on 19. 9. 30..
//

#ifndef GENGINE_DEBUGRENDERER_H
#define GENGINE_DEBUGRENDERER_H

#include "gengine/Camera.h"
#include "gengine/Shader.h"

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glad/glad.h>
#include <span.hpp>

struct ImStringData {
    glm::vec3 color;
    float posX;
    float posY;
    float scaling;
    std::string text;
    bool centered;
};

struct ImPointData {
    glm::vec3 position;
    glm::vec3 color;
    float size;
    bool depthEnabled;
};

struct ImLineData {
    glm::vec3 from;
    glm::vec3 to;
    glm::vec3 color;
    bool depthEnabled;
};

union ImDrawVertex {
    struct {
        glm::vec3 pos;
        glm::vec3 color;
        float size;
    } point;

    struct {
        glm::vec3 pos;
        glm::vec3 color;
    } line;

    struct {
        glm::vec2 pos;
        glm::vec2 uv;
        glm::vec3 color;
    } glyph;
};

#define IM_VERTEX_BUFFER_SIZE (1 << 18)

struct DebugRenderer {

    DebugRenderer(Camera* camera = nullptr) : camera(camera) {}

    void setCamera(Camera* camera) { this->camera = camera; }

    void init();
    void reserveBuffers(int stringBufSize, int pointBufSize, int lineBufSize);
    void render();
    void endFrame();

    void drawDebugPoints(bool depthEnabled);
    void drawDebugLines(bool depthEnabled);

    // Immediate-mode rendering functions
    void drawPoint(glm::vec3 pos, glm::vec3 color, float size, bool depthEnabled);
    void drawLine(glm::vec3 from, glm::vec3 to, glm::vec3 color, bool depthEnabled);
    void drawText(std::string_view str, glm::vec3 pos, glm::vec3 color, float scaling);
    void drawAxisTriad(glm::mat4 transform, float size, float length, bool depthEnabled);
    void drawArrow(glm::vec3 from, glm::vec3 to, glm::vec3 color, float size, bool depthEnabled);
    void drawCross(glm::vec3 center, float length, bool depthEnabled);
    void drawCircle(glm::vec3 center, glm::vec3 planeNormal, glm::vec3 color, float radius, int numSteps, bool depthEnabled);
    void drawPlane(glm::vec3 center, glm::vec3 planeColor,
            glm::vec3 planeNormal, glm::vec3 normalVecColor, float planeScale, float normalVecScale, bool depthEnabled);
    void drawSphere(glm::vec3 center, glm::vec3 color, float radius, bool depthEnabled);
    void drawCone(glm::vec3 apex, glm::vec3 dir, glm::vec3 color,
                    float baseRadius, float apexRadius, bool depthEnabled);
    void drawBox(std::span<glm::vec3> points, glm::vec3 color, bool depthEnabled);
    void drawBox(glm::vec3 center, glm::vec3 color, float width, float height, float depth, bool depthEnabled);
    void drawFrustrum(glm::mat4 invClipMatrix, glm::vec3 color, bool depthEnabled);
    void drawVertexNormal(glm::vec3 origin, glm::vec3 normal, float length, bool depthEnabled);
    void drawTangentBasis(glm::vec3 origin, glm::vec3 normal, glm::vec3 tangent, glm::vec3 bitangent, float lengths, bool depthEnabled);
    void drawXZSquareGrid(float mins, float maxs, float y, float step, glm::vec3 color, bool depthEnabled);

private:
    void pushPointVert(const ImPointData& point);
    void pushLineVert(const ImLineData& line);

    void flushPointVerts(bool depthEnabled);
    void flushLineVerts(bool depthEnabled);

    Camera* camera;

    std::vector<ImStringData> stringData;
    std::vector<ImPointData> pointData;
    std::vector<ImLineData> lineData;

    std::vector<ImDrawVertex> vertexBuffer;
    int vertexBufferUsed = 0;

    Ref<Shader> imPointShader;
    Ref<Shader> imLineShader;

    GLuint pointVao;
    GLuint lineVao;

    GLuint vbo;
};

#endif //GENGINE_DEBUGRENDERER_H
