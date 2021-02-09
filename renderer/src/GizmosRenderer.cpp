//
// Created by lasagnaphil on 19. 9. 30..
//

#include "gengine/GizmosRenderer.h"

void GizmosRenderer::render() {
    Shaders::line3D->use();
    Shaders::line3D->setCamera(camera);
    Shaders::point->use();
    Shaders::point->setCamera(camera);

    for (auto& command : lineRenderCommands) {
        glBindVertexArray(command.mesh->vao);

        auto& material = command.material;
        if (material->drawLines) {
            Shaders::line3D->use();
            Shaders::line3D->setMat4("model", command.modelMatrix);
            Shaders::line3D->setVec4("color", material->lineColor);
#ifndef __APPLE__
            glLineWidth(material->lineWidth);
#endif
            glDrawArrays(material->lineType, 0, command.mesh->positions.size());
        }
        if (material->drawPoints) {
            Shaders::point->use();
            Shaders::point->setMat4("model", command.modelMatrix);
            Shaders::point->setVec4("color", material->pointColor);
            glPointSize(material->pointSize);
            glDrawArrays(GL_POINTS, 0, command.mesh->positions.size());
        }

        glBindVertexArray(0);
    }
    lineRenderCommands.clear();
}

