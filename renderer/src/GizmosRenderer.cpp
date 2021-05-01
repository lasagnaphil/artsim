//
// Created by lasagnaphil on 19. 9. 30..
//

#include "gengine/GizmosRenderer.h"

#include "shaders/line3d.vert.h"
#include "shaders/line3d.frag.h"
#include "shaders/point.vert.h"
#include "shaders/point.frag.h"

void GizmosRenderer::init() {
    if (camera == nullptr) {
        fmt::print(stderr, "Camera not attached to PhongRenderer!\n");
        exit(EXIT_FAILURE);
    }
    line3DShader = Shader::fromString("line3d", line3d_vert_shader, line3d_frag_shader);
    pointShader = Shader::fromString("point", point_vert_shader, point_frag_shader);
}


void GizmosRenderer::render() {
    line3DShader->use();
    line3DShader->setCamera(camera);
    pointShader->use();
    pointShader->setCamera(camera);

    for (auto& command : lineRenderCommands) {
        glBindVertexArray(command.mesh->vao);

        auto& material = command.material;
        if (material->drawLines) {
            line3DShader->use();
            line3DShader->setMat4("model", command.modelMatrix);
            line3DShader->setVec4("color", material->lineColor);
#ifndef __APPLE__
            glLineWidth(material->lineWidth);
#endif
            glDrawArrays(material->lineType, 0, command.mesh->positions.size());
        }
        if (material->drawPoints) {
            pointShader->use();
            pointShader->setMat4("model", command.modelMatrix);
            pointShader->setVec4("color", material->pointColor);
            glPointSize(material->pointSize);
            glDrawArrays(GL_POINTS, 0, command.mesh->positions.size());
        }

        glBindVertexArray(0);
    }
    lineRenderCommands.clear();
}
