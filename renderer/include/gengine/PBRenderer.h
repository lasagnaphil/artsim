//
// Created by lasagnaphil on 19. 9. 28..
//

#ifndef GENGINE_PBRENDERER_H
#define GENGINE_PBRENDERER_H

#define NUM_PBR_POINT_LIGHTS 16
#define NUM_PBR_SPOT_LIGHTS 8

#include <artsim/math/box.h>

#include "gengine/Shader.h"
#include "gengine/Texture.h"
#include "gengine/Mesh.h"
#include "gengine/Colors.h"
#include "gengine/DebugRenderer.h"

#include <array>
#include "artsim/math/rect.h"

#include <glm/gtc/type_ptr.hpp>

// TODO: Add normal texture (needed for normal mapping)
struct PBRMaterial {
    static Ref<Texture> defaultTexture;

    Ref<Texture> texAlbedo = {};
    Ref<Texture> texMetallic = {};
    Ref<Texture> texRoughness = {};
    Ref<Texture> texAO = {};

    glm::vec3 albedo = glm::vec3(1, 1, 1);
    float metallic = 1.0f;
    float roughness = 1.0f;
    float ao = 1.0f;

    bool transparent = false;
    float alpha = 1.0f;

    static Ref<PBRMaterial> quick(
            const std::string& albedo,
            const std::string& metallic,
            const std::string& roughness,
            const std::string& ao);

    static Ref<PBRMaterial> quick(glm::vec3 color);

    static Ref<PBRMaterial> fromOBJ(const tinyobj::material_t& tmat, const char* directory = nullptr);
};

struct PBRDirLight {
    alignas(16) glm::vec3 direction;
    alignas(16) glm::vec3 color;
    uint32_t enabled = false;
};

struct PBRPointLight {
    alignas(16) glm::vec3 position;
    alignas(16) glm::vec3 color;
    uint32_t enabled = false;
};

struct PBRSpotLight {
    alignas(16) glm::vec3 position;
    alignas(16) glm::vec3 direction;
    alignas(16) glm::vec3 color;

    float cutOff;
    float outerCutOff;

    uint32_t enabled = false;
};

struct PBRLights {
    alignas(16) PBRDirLight dir;
    alignas(16) std::array<PBRPointLight, NUM_PBR_POINT_LIGHTS> point;
    alignas(16) std::array<PBRSpotLight, NUM_PBR_SPOT_LIGHTS> spot;
};

struct PBRCommand {
    Ref<Mesh> mesh;
    Ref<PBRMaterial> material;
    glm::mat4 modelMatrix;
};

class PBRenderer {
public:
    PBRenderer(Camera* camera = nullptr);

    void setCamera(Camera* camera) {
        this->camera = camera;
    }

    void setShadowSettings(glmx::tbox<3, float> projVolume, glm::ivec2 shadowFBSize) {
        this->dirLightProjVolume = projVolume;
        this->shadowFramebufferSize = shadowFBSize;
    }

    void init();

    void queueRender(const PBRCommand& command) {
        if (command.material->transparent) {
            renderTransparentCommands.push_back(command);
        }
        else {
            renderSolidCommands.push_back(command);
        }
    }

    void render(bool shadows = false);

    void renderImGui();

    DebugRenderer* getDebugRenderer() { return &debugRenderer; }

    glm::vec3 skyColor = {1.0f, 1.0f, 1.0f};
    float exposure = 5.0f;

    glmx::tbox<3, float> dirLightProjVolume = {
            {-10.f, -10.f, 0.f}, {10.f, 10.f, 100.f}
    };
    glm::ivec2 shadowFramebufferSize = {2048, 2048};

    PBRLights lights;

    bool dirLightFollowingCamera = false;

private:
    glm::mat4 calcDirLightSpaceMatrix();
    void setLightingUniforms(Ref<Shader> shader, bool shadows);
    void renderPass(Ref<Shader> shader, std::vector<PBRCommand>& commands);

    // Includes a debug renderer, since we need to render debug info in opqaue pass
    DebugRenderer debugRenderer;

    GLuint quadVAO, quadVBO;

    GLuint screenFBO;
    GLuint opaqueTexture, depthTexture, accumTexture, revealTexture;

    GLuint depthMapFBO;
    GLuint depthMap;

    std::vector<PBRCommand> renderSolidCommands;
    std::vector<PBRCommand> renderTransparentCommands;

    GLuint lightUBO;

    Camera* camera;

    Ref<Shader> depthShader, pbrSolidShader, pbrTransparentShader, compositeShader, screenShader;
};

#endif //GENGINE_PBRENDERER_H
