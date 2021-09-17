//
// Created by lasagnaphil on 21. 9. 8..
//

#include "gengine/ColoredMeshRenderer.h"
#include "shaders/coloredmesh.vert.h"
#include "shaders/coloredmesh.frag.h"

void ColoredMeshRenderer::init() {
    coloredMeshShader = Shader::fromString("colored_mesh", coloredmesh_vert_shader, coloredmesh_frag_shader);
}

void ColoredMeshRenderer::renderOpaque() {
    coloredMeshShader->use();
    coloredMeshShader->setCamera(camera);
    for (const ColoredMeshRenderCommand& command : opaqueCommands) {
        coloredMeshShader->setMat4("model", command.modelMatrix);
        glBindVertexArray(command.mesh->vao);
        glDrawArrays(command.mesh->drawType, 0, command.mesh->vertices.size());
    }
    glBindVertexArray(0);
    opaqueCommands.clear();
}

void ColoredMeshRenderer::renderTransparent() {
    coloredMeshShader->use();
    coloredMeshShader->setCamera(camera);
    for (const ColoredMeshRenderCommand& command : transparentCommands) {
        coloredMeshShader->setMat4("model", command.modelMatrix);
        glBindVertexArray(command.mesh->vao);
        glDrawArrays(command.mesh->drawType, 0, command.mesh->vertices.size());
    }
    glBindVertexArray(0);
    transparentCommands.clear();
}
