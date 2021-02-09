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
        return material;
    }
};

struct PBRDirLight {
    glm::vec3 direction;
    glm::vec3 color;
    uint32_t enabled = false;
};

struct PBRPointLight {
    glm::vec3 position;
    glm::vec3 color;
    uint32_t enabled = false;
};

struct PBRSpotLight {
    glm::vec3 position;
    glm::vec3 direction;
    glm::vec3 color;

    float cutOff;
    float outerCutOff;

    uint32_t enabled = false;
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
        renderCommands.push_back(command);
    }

    void render();

    void renderImGui();

    PBRDirLight dirLight = {
            glm::normalize(glm::vec3 {2.0f, -3.0f, 2.0f}),
            {1.0f, 1.0f, 1.0f},
            true
    };

    std::array<PBRPointLight, NUM_PBR_POINT_LIGHTS> pointLights;
    std::array<PBRSpotLight, NUM_PBR_SPOT_LIGHTS> spotLights;

    glmx::box dirLightProjVolume = {
            {-10.f, -10.f, 0.f}, {10.f, 10.f, 100.f}
    };
    glm::ivec2 shadowFramebufferSize = {2048, 2048};

private:
    void renderPass(Ref<Shader> shader);

    GLuint depthMapFBO;
    GLuint depthMap;

    std::vector<PBRCommand> renderCommands;

    GLuint dirLightUBO;
    GLuint pointLightUBO;
    GLuint spotLightUBO;

    Camera* camera;

    Ref<Shader> pbrShader;
    Ref<Shader> depthShader;
};

#endif //GENGINE_PBRENDERER_H
