//
// Created by lasagnaphil on 19. 3. 17.
//

#ifndef MOTION_EDITING_PHONGRENDERER_H
#define MOTION_EDITING_PHONGRENDERER_H

#include "gengine/Texture.h"
#include "gengine/Mesh.h"

#include <imgui.h>
#include <glm/gtc/matrix_transform.hpp>
#include <array>

#define NR_POINT_LIGHTS 16
#define NR_SPOT_LIGHTS 8

struct Rect3f {
    float left, right, bottom, top, zNear, zFar;
};

struct PhongRenderCommand {
    Ref<Mesh> mesh;
    Ref<PhongMaterial> material;
    glm::mat4 modelMatrix;
};

struct PhongMaterial {
    glm::vec4 ambient;
    glm::vec4 diffuse;
    glm::vec4 specular;
    float shininess = 32.0f;
    Ref<Texture> texDiffuse = {};
    Ref<Texture> texSpecular = {};
};

struct DirLight {
    bool enabled = false;

    glm::vec3 direction = {};

    glm::vec4 ambient = {};
    glm::vec4 diffuse = {};
    glm::vec4 specular = {};

    float intensity = 32.0f;
};

struct PointLight {
    bool enabled = false;

    glm::vec3 position = {};

    float constant;
    float linear;
    float quadratic;

    glm::vec4 ambient = {};
    glm::vec4 diffuse = {};
    glm::vec4 specular = {};

    float intensity = 32.0f;
};

struct SpotLight {
    bool enabled = false;

    glm::vec3 position = {};
    glm::vec3 direction = {};
    float cutOff;
    float outerCutOff;

    glm::vec4 ambient = {};
    glm::vec4 diffuse = {};
    glm::vec4 specular = {};

    float constant;
    float linear;
    float quadratic;

    float intensity = 32.0f;
};

class PhongRenderer {
public:
    bool viewDepthBufferDebug = false;

    PhongRenderer(Rect3f projVolume = {-5.f, 5.f, -5.f, 5.f, 0.f, 1000.f},
            glm::ivec2 shadowFBSize = {1024, 1024},
            Camera* camera = nullptr);

    void setCamera(Camera* camera) {
        this->camera = camera;
    }

    void init();

    void queueRender(const PhongRenderCommand& command) {
        renderCommands.push_back(command);
    }

    void render();

    void renderImGui();

private:
    void renderPass(Ref<Shader> shader);

    void renderDepthMapDebug();

    GLuint defaultFBO = 0;

    GLuint depthMapFBO;
    GLuint depthMap;

    std::vector<PhongRenderCommand> renderCommands;

    DirLight dirLight;
    std::array<PointLight, NR_POINT_LIGHTS> pointLights;
    std::array<SpotLight, NR_SPOT_LIGHTS> spotLights;

    Camera* camera;

    Rect3f dirLightProjVolume;
    glm::ivec2 shadowFramebufferSize;

    Ref<Shader> depthShader;
    Ref<Shader> phongShader;
    Ref<Shader> debugDepthShader;
};

#endif //MOTION_EDITING_PHONGRENDERER_H
