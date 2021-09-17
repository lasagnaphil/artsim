//
// Created by lasagnaphil on 21. 9. 8..
//

#ifndef EOS_SCAN_TO_HUMAN_COLOREDMESHRENDERER_H
#define EOS_SCAN_TO_HUMAN_COLOREDMESHRENDERER_H

#include <gengine/ColoredMesh.h>

struct ColoredMeshRenderCommand {
    Ref<ColoredMesh> mesh;
    glm::mat4 modelMatrix;
};

class ColoredMeshRenderer {
public:
    ColoredMeshRenderer(Camera* camera = nullptr) : camera(camera) {}

    void init();

    void setCamera(Camera* camera) {
        this->camera = camera;
    }

    void queueRender(Ref<ColoredMesh> mesh, glm::mat4 modelMatrix, bool transparent = false) {
        if (transparent) {
            transparentCommands.push_back({mesh, modelMatrix});
        }
        else {
            opaqueCommands.push_back({mesh, modelMatrix});
        }
    }

    void renderOpaque();
    void renderTransparent();

private:
    std::vector<ColoredMeshRenderCommand> opaqueCommands;
    std::vector<ColoredMeshRenderCommand> transparentCommands;
    Camera* camera;

    Ref<Shader> coloredMeshShader;
};

#endif //EOS_SCAN_TO_HUMAN_COLOREDMESHRENDERER_H
