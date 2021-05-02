//
// Created by lasagnaphil on 19. 9. 28..
//

#ifndef GENGINE_PBRENDERER_H
#define GENGINE_PBRENDERER_H

#define NUM_PBR_POINT_LIGHTS 16
#define NUM_PBR_SPOT_LIGHTS 8

#include "gengine/Shader.h"
#include "gengine/Texture.h"
#include "gengine/Mesh.h"
#include "gengine/Colors.h"

#include <array>
#include "artsim/math/rect.h"

// TODO: Add normal texture (needed for normal mapping)
struct PBRMaterial {
    Ref<Texture> texAlbedo;
    Ref<Texture> texMetallic;
    Ref<Texture> texRoughness;
    Ref<Texture> texAO;
    bool transparent = false;
    float alpha = 1.0f;

    static Ref<PBRMaterial> quick(
            const std::string& albedo,
            const std::string& metallic,
            const std::string& roughness,
            const std::string& ao);

    static Ref<PBRMaterial> quick(glm::vec3 color) {
        static Ref<Texture> defaultAO = {};
        static Ref<Texture> defaultMetallic = {};
        static Ref<Texture> defaultRoughness = {};
        if (!defaultAO) {
            defaultAO = Texture::fromSingleColor({1.0f, 0.0f, 0.0f});
        }
        if (!defaultMetallic) {
            defaultMetallic = Texture::fromSingleColor({0.0f, 0.0f, 0.0f});
        }
        if (!defaultRoughness) {
            defaultRoughness = Texture::fromSingleColor({0.0f, 0.0f, 0.0f});
        }
        Ref<PBRMaterial> material = Resources::make<PBRMaterial>();
        material->texAlbedo = Texture::fromSingleColor(color);
        material->texAO = defaultAO;
        material->texMetallic = defaultMetallic;
        material->texRoughness = defaultRoughness;
        material->alpha = 1.0f;
        return material;
    }
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

    void setShadowSettings(glmx::box projVolume, glm::ivec2 shadowFBSize) {
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

    glm::vec3 skyColor = {1.0f, 1.0f, 1.0f};
    float exposure = 5.0f;

    glmx::box dirLightProjVolume = {
            {-10.f, -10.f, 0.f}, {10.f, 10.f, 100.f}
    };
    glm::ivec2 shadowFramebufferSize = {2048, 2048};

    PBRLights lights;

private:
    glm::mat4 calcDirLightSpaceMatrix();
    void setLightingUniforms(Ref<Shader> shader, bool shadows);
    void renderPass(Ref<Shader> shader, std::vector<PBRCommand>& commands);

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
