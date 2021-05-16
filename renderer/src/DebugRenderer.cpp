//
// Created by lasagnaphil on 19. 9. 30..
//

#include <glm/ext/scalar_constants.hpp>
#include <glm/trigonometric.hpp>
#include "gengine/DebugRenderer.h"
#include "gengine/Shader.h"
#include "gengine/Colors.h"
#include <fmt/core.h>

#include "shaders/colorpoint3d.vert.h"
#include "shaders/colorpoint3d.frag.h"
#include "shaders/colorline3d.vert.h"
#include "shaders/colorline3d.frag.h"

#include <tuple>

void DebugRenderer::init() {
    if (camera == nullptr) {
        fmt::print(stderr, "Camera not attached to PhongRenderer!\n");
        exit(EXIT_FAILURE);
    }
    vertexBuffer.resize(IM_VERTEX_BUFFER_SIZE);

    imPointShader = Resources::make<Shader>();
    imPointShader->compileFromString(colorpoint3d_vert_shader, colorpoint3d_frag_shader);
    imPointShader->use();
    imPointShader->setMat4("model", glm::mat4(1.0f));

    imLineShader = Resources::make<Shader>();
    imLineShader->compileFromString(colorline3d_vert_shader, colorline3d_frag_shader);
    imLineShader->use();
    imLineShader->setMat4("model", glm::mat4(1.0f));

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(ImDrawVertex) * IM_VERTEX_BUFFER_SIZE, nullptr, GL_DYNAMIC_DRAW);

    glGenVertexArrays(1, &pointVao);
    glBindVertexArray(pointVao);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(ImDrawVertex), (void*)offsetof(ImDrawVertex, point.pos));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(ImDrawVertex), (void*)offsetof(ImDrawVertex, point.color));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(ImDrawVertex), (void*)offsetof(ImDrawVertex, point.size));

    glGenVertexArrays(1, &lineVao);
    glBindVertexArray(lineVao);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(ImDrawVertex), (void*)offsetof(ImDrawVertex, line.pos));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(ImDrawVertex), (void*)offsetof(ImDrawVertex, line.color));

    glBindVertexArray(0);
}

void DebugRenderer::reserveBuffers(int stringBufSize, int pointBufSize, int lineBufSize) {
    stringData.reserve(stringBufSize);
    pointData.reserve(pointBufSize);
    lineData.reserve(lineBufSize);
}

void DebugRenderer::render() {
    drawDebugPoints();
    drawDebugLines();
}

void DebugRenderer::drawDebugPoints() {
    imPointShader->use();
    imPointShader->setCamera(camera);
    if (pointData.empty()) return;

    int numDepthlessPoints = 0;
    for (auto& point : pointData) {
        if (point.depthEnabled) {
            pushPointVert(point);
        }
        numDepthlessPoints += !point.depthEnabled;
    }
    flushPointVerts(true);

    if (numDepthlessPoints > 0) {
        for (auto& point : pointData) {
            if (!point.depthEnabled) {
                pushPointVert(point);
            }
        }
        flushPointVerts(false);
    }
    pointData.clear();
}

void DebugRenderer::drawDebugLines() {
    imLineShader->use();
    imLineShader->setCamera(camera);
    if (lineData.empty()) return;

    int numDepthlessLines = 0;
    for (auto& line : lineData) {
        if (line.depthEnabled) {
            pushLineVert(line);
        }
        numDepthlessLines += !line.depthEnabled;
    }
    flushLineVerts(true);

    if (numDepthlessLines > 0) {
        for (auto& line : lineData) {
            if (!line.depthEnabled) {
                pushLineVert(line);
            }
        }
        flushLineVerts(false);
    }
    lineData.clear();
}

void DebugRenderer::pushPointVert(const ImPointData &point) {
    if (vertexBufferUsed >= vertexBuffer.size()) {
        flushPointVerts(point.depthEnabled);
    }
    ImDrawVertex& v = vertexBuffer[vertexBufferUsed++];
    v.point.pos = point.position;
    v.point.color = point.color;
    v.point.size = point.size;
}

void DebugRenderer::pushLineVert(const ImLineData &line) {
    if (vertexBufferUsed >= vertexBuffer.size()) {
        flushLineVerts(line.depthEnabled);
    }
    ImDrawVertex& v0 = vertexBuffer[vertexBufferUsed++];
    ImDrawVertex& v1 = vertexBuffer[vertexBufferUsed++];
    v0.line.pos = line.from;
    v0.line.color = line.color;
    v1.line.pos = line.to;
    v1.line.color = line.color;
}

void DebugRenderer::flushPointVerts(bool depthEnabled) {
    if (vertexBufferUsed == 0) return;

    if (depthEnabled)
        glEnable(GL_DEPTH_TEST);
    else
        glDisable(GL_DEPTH_TEST);

    glBindVertexArray(pointVao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertexBufferUsed * sizeof(ImDrawVertex), vertexBuffer.data());
    glDrawArrays(GL_POINTS, 0, vertexBufferUsed);
    glBindVertexArray(0);

    if (!depthEnabled) glEnable(GL_DEPTH_TEST);

    vertexBufferUsed = 0;
}

void DebugRenderer::flushLineVerts(bool depthEnabled) {
    if (vertexBufferUsed == 0) return;

    if (depthEnabled)
        glEnable(GL_DEPTH_TEST);
    else
        glDisable(GL_DEPTH_TEST);

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);

    glBindVertexArray(lineVao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertexBufferUsed * sizeof(ImDrawVertex), vertexBuffer.data());
    glDrawArrays(GL_LINES, 0, vertexBufferUsed);
    glBindVertexArray(0);

    if (!depthEnabled) glEnable(GL_DEPTH_TEST);

    vertexBufferUsed = 0;
}

inline std::tuple<glm::vec3, glm::vec3> getOrthogonalBasis(glm::vec3 v) {
    glm::vec3 left, up;
    if (glm::abs(v.z) >= 0.7f) {
        float lenSqr = v.y * v.y + v.z * v.z;
        float invLen = 1 / sqrtf(lenSqr);
        up.x = 0.0f;
        up.y = v.z * invLen;
        up.z = -v.y * invLen;
        left.x = lenSqr * invLen;
        left.y = -v.x * up.z;
        left.z = v.x * up.y;
    }
    else {
        float lenSqr = v.x * v.x + v.y * v.y;
        float invLen = 1 / sqrtf(lenSqr);
        left.x = -v.y * invLen;
        left.y = v.x * invLen;
        left.z = 0.0f;
        up.x = -v.z * left.y;
        up.y = v.z * left.x;
        up.z = lenSqr * invLen;
    }
    return {left, up};
}

void DebugRenderer::drawPoint(glm::vec3 pos, glm::vec3 color, float size, bool depthEnabled) {
    pointData.push_back(ImPointData {pos, color, size, depthEnabled});
}

void DebugRenderer::drawLine(glm::vec3 from, glm::vec3 to, glm::vec3 color, bool depthEnabled) {
    lineData.push_back(ImLineData {from, to, color, depthEnabled});
}

void DebugRenderer::drawText(std::string_view str, glm::vec3 pos, glm::vec3 color, float scaling) {
    assert(false && "Drawing text not supported yet!");
}

void DebugRenderer::drawAxisTriad(glm::mat4 transform, float size, float length, bool depthEnabled) {
    glm::vec3 origin = {0.0f, 0.0f, 0.0f};
    glm::vec3 xEnd = {length, 0.0f, 0.0f};
    glm::vec3 yEnd = {0.0f, length, 0.0f};
    glm::vec3 zEnd = {0.0f, 0.0f, length};

    glm::vec3 p0 = transform * glm::vec4(origin, 1.0f);
    glm::vec3 p1 = transform * glm::vec4(xEnd, 1.0f);
    glm::vec3 p2 = transform * glm::vec4(yEnd, 1.0f);
    glm::vec3 p3 = transform * glm::vec4(zEnd, 1.0f);

    drawArrow(p0, p1, colors::Red, size, depthEnabled);
    drawArrow(p0, p2, colors::Green, size, depthEnabled);
    drawArrow(p0, p3, colors::Blue, size, depthEnabled);
}

void DebugRenderer::drawArrow(glm::vec3 from, glm::vec3 to, glm::vec3 color, float size, bool depthEnabled) {
    static const float arrowStep = 30.0f; // In degrees
    static const float arrowSin[45] = {
            0.0f, 0.5f, 0.866025f, 1.0f, 0.866025f, 0.5f, -0.0f, -0.5f, -0.866025f,
            -1.0f, -0.866025f, -0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    };
    static const float arrowCos[45] = {
            1.0f, 0.866025f, 0.5f, -0.0f, -0.5f, -0.866026f, -1.0f, -0.866025f, -0.5f, 0.0f,
            0.5f, 0.866026f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    };

    // Body line:
    drawLine(from, to, color, depthEnabled);

    // Aux vectors to compute the arrowhead:
    glm::vec3 forward = glm::normalize(to - from);
    auto [right, up] = getOrthogonalBasis(forward);
    forward *= size;

    // Arrowhead is a cone (sin/cos tables used here):
    float degrees = 0.0f;
    for (int i = 0; degrees < 360.0f; degrees += arrowStep, ++i)
    {
        glm::vec3 v1 = to - forward + 0.5f * size * arrowCos[i] * right;
        v1 += 0.5f * size * arrowSin[i] * up;

        glm::vec3 v2 = to - forward + 0.5f * size * arrowCos[i+1] * right;
        v2 += 0.5f * size * arrowSin[i+1] * up;

        drawLine(v1, to, color, depthEnabled);
        drawLine(v2, v2, color, depthEnabled);
    }
}

void DebugRenderer::drawCross(glm::vec3 center, float length, bool depthEnabled) {
    float hl = length * 0.5f;
    drawLine({center.x - hl, center.y, center.z}, {center.x + hl, center.y, center.z},
            colors::Red, depthEnabled);
    drawLine({center.x, center.y - hl, center.z}, {center.x, center.y + hl, center.z},
             colors::Green, depthEnabled);
    drawLine({center.x, center.y, center.z - hl}, {center.x, center.y, center.z + hl},
             colors::Blue, depthEnabled);
}

void DebugRenderer::drawCircle(glm::vec3 center, glm::vec3 planeNormal, glm::vec3 color, float radius, int numSteps,
                               bool depthEnabled) {

    auto [left, up] = getOrthogonalBasis(planeNormal);
    left *= radius;
    up *= radius;

    glm::vec3 lastPoint = center + up;

    for (int i = 1; i < numSteps; ++i) {
        const float radians = M_2_PI * i / numSteps;

        glm::vec3 vs = left * sinf(radians);
        glm::vec3 vc = up * cosf(radians);
        glm::vec3 point = center + vs + vc;

        drawLine(lastPoint, point, color, depthEnabled);
        lastPoint = point;
    }
}

void DebugRenderer::drawPlane(glm::vec3 center, glm::vec3 planeNormal, glm::vec3 planeColor, glm::vec3 normalVecColor, float planeScale,
                              float normalVecScale, bool depthEnabled) {

    auto [tangent, bitangent] = getOrthogonalBasis(planeNormal);
    glm::vec3 v1 = center - tangent * planeScale - bitangent * planeScale;
    glm::vec3 v2 = center + tangent * planeScale - bitangent * planeScale;
    glm::vec3 v3 = center + tangent * planeScale + bitangent * planeScale;
    glm::vec3 v4 = center - tangent * planeScale + bitangent * planeScale;

    drawLine(v1, v2, planeColor, depthEnabled);
    drawLine(v2, v3, planeColor, depthEnabled);
    drawLine(v3, v4, planeColor, depthEnabled);
    drawLine(v4, v1, planeColor, depthEnabled);
}

void DebugRenderer::drawSphere(glm::vec3 center, glm::vec3 color, float radius, bool depthEnabled) {
    static const int stepSize = 15;
    int cacheLen = 360 / stepSize;
    std::vector<glm::vec3> cache(cacheLen);
    glm::vec3 radiusVec = {0.0f, 0.0f, radius};
    cache[0] += center + radiusVec;

    for (int n = 1; n < cache.size(); ++n)
    {
        cache[n] = cache[0];
    }

    glm::vec3 lastPoint, temp;
    for (int i = stepSize; i <= 360; i += stepSize)
    {
        const float s = sinf(glm::radians((float)i));
        const float c = cosf(glm::radians((float)i));

        lastPoint.x = center.x;
        lastPoint.y = center.y + radius * s;
        lastPoint.z = center.z + radius * c;

        for (int n = 0, j = stepSize; j <= 360; j += stepSize, ++n)
        {
            temp.x = center.x + sinf(glm::radians((float)j)) * radius * s;
            temp.y = center.y + cosf(glm::radians((float)j)) * radius * s;
            temp.z = lastPoint.z;

            drawLine(lastPoint, temp, color, depthEnabled);
            drawLine(lastPoint, cache[n], color, depthEnabled);

            cache[n] = lastPoint;
            lastPoint = temp;
        }
    }
}

void DebugRenderer::drawCone(glm::vec3 apex, glm::vec3 dir, glm::vec3 color, float baseRadius, float apexRadius,
                             bool depthEnabled) {

    assert(false && "drawCone not implemented yet!");
}

void DebugRenderer::drawBox(nonstd::span <glm::vec3> points, glm::vec3 color, bool depthEnabled) {
    assert(points.size() == 8);

    for (int i = 0; i < 4; ++i)
    {
        drawLine(points[i], points[(i + 1) % 4], color, depthEnabled);
        drawLine(points[4 + i], points[4 + ((i + 1) % 4)], color, depthEnabled);
        drawLine(points[i], points[4 + i], color, depthEnabled);
    }
}

void
DebugRenderer::drawBox(glm::vec3 center, glm::vec3 color, float width, float height, float depth, bool depthEnabled) {
    // Create all the 8 points:
    glm::vec3 points[8];
#define DD_BOX_V(v, op1, op2, op3) \
    v.x = center.x op1 width; \
    v.y = center.y op2 height; \
    v.z = center.z op3 depth
    DD_BOX_V(points[0], -, +, +);
    DD_BOX_V(points[1], -, +, -);
    DD_BOX_V(points[2], +, +, -);
    DD_BOX_V(points[3], +, +, +);
    DD_BOX_V(points[4], -, -, +);
    DD_BOX_V(points[5], -, -, -);
    DD_BOX_V(points[6], +, -, -);
    DD_BOX_V(points[7], +, -, +);
#undef DD_BOX_V

    drawBox(points, color, depthEnabled);
}

void DebugRenderer::drawFrustrum(glm::mat4 invClipMatrix, glm::vec3 color, bool depthEnabled) {
    // Start with the standard clip volume, then bring it back to world space.
    static const glm::vec3 planes[8] = {
            // near plane
            { -1.0f, -1.0f, -1.0f }, {  1.0f, -1.0f, -1.0f },
            {  1.0f,  1.0f, -1.0f }, { -1.0f,  1.0f, -1.0f },
            // far plane
            { -1.0f, -1.0f,  1.0f }, {  1.0f, -1.0f,  1.0f },
            {  1.0f,  1.0f,  1.0f }, { -1.0f,  1.0f,  1.0f }
    };

    glm::vec3 points[8];

    // Transform the planes by the inverse clip matrix:
    for (int i = 0; i < 8; ++i)
    {
        glm::vec4 hCoords = invClipMatrix * glm::vec4(planes[i], 1.0f);
        if (hCoords.w < glm::epsilon<float>()) {
            return;
        }
        points[i] = {hCoords.x / hCoords.w, hCoords.y / hCoords.w, hCoords.z / hCoords.w};
    }

    // Connect the dots:
    drawBox(points, color, depthEnabled);
}

void DebugRenderer::drawVertexNormal(glm::vec3 origin, glm::vec3 normal, float length, bool depthEnabled) {
    drawLine(origin, origin + normal * length, colors::White, depthEnabled);
}

void DebugRenderer::drawTangentBasis(glm::vec3 origin, glm::vec3 normal, glm::vec3 tangent, glm::vec3 bitangent,
                                     float lengths, bool depthEnabled) {
    glm::vec3 N = normal * lengths + origin;
    glm::vec3 T = tangent * lengths + origin;
    glm::vec3 B = bitangent * lengths + origin;

    drawLine(origin, N, colors::White, depthEnabled);
    drawLine(origin, T, colors::Yellow, depthEnabled);
    drawLine(origin, B, colors::Magenta, depthEnabled);
}

void DebugRenderer::drawXZSquareGrid(float mins, float maxs, float y, float step, glm::vec3 color, bool depthEnabled) {
    glm::vec3 from, to;
    for (float i = mins; i <= maxs; i += step)
    {
        // Horizontal line (along the X)
        drawLine({mins, y, i}, {maxs, y, i}, color, depthEnabled);

        // Vertical line (along the Z)
        drawLine({i, y, mins}, {i, y, maxs}, color, depthEnabled);
    }
}


