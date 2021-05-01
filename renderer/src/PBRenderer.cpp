//
// Created by lasagnaphil on 19. 11. 17..
//

#include "gengine/PBRenderer.h"

#include <imgui.h>
#include <fmt/core.h>
#include <glm/gtc/matrix_transform.hpp>

#include "shaders/depth.vert.h"
#include "shaders/depth.frag.h"
#include "shaders/pbr.vert.h"
#include "shaders/pbr_solid.frag.h"
#include "shaders/pbr_transparent.frag.h"

static float quadVertices[] = {
        // positions        // uv
        -1.0f, -1.0f, 0.0f,	0.0f, 0.0f,
        1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
        1.0f,  1.0f, 0.0f, 1.0f, 1.0f,

        1.0f,  1.0f, 0.0f, 1.0f, 1.0f,
        -1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
        -1.0f, -1.0f, 0.0f, 0.0f, 0.0f
};

Ref<PBRMaterial>
PBRMaterial::quick(const std::string &albedo, const std::string &metallic, const std::string &roughness,
                   const std::string &ao) {

    Ref<Image> albedoImage = Image::fromFile(albedo);
    Ref<Image> metallicImage = Image::fromFile(metallic);
    Ref<Image> roughnessImage = Image::fromFile(roughness);
    Ref<Image> aoImage = Image::fromFile(ao);

    Ref<PBRMaterial> mat = Resources::make<PBRMaterial>();

    mat->texAlbedo = Texture::fromImage(albedoImage);
    mat->texMetallic = Texture::fromImage(metallicImage);
    mat->texRoughness = Texture::fromImage(roughnessImage);
    mat->texAO = Texture::fromImage(aoImage);
    mat->alpha = 1.0f;

    return mat;
}

PBRenderer::PBRenderer(Camera *camera) :
        camera(camera) {
}

void PBRenderer::init() {
    if (camera == nullptr) {
        fmt::print(stderr, "Camera not attached to PhongRenderer!\n");
        exit(EXIT_FAILURE);
    }

    pbrSolidShader = Shader::fromString("pbr_solid", pbr_vert_shader, pbr_solid_frag_shader);
    pbrTransparentShader = Shader::fromString("pbr_transparent", pbr_vert_shader, pbr_transparent_frag_shader);
    depthShader = Shader::fromString("depth", depth_vert_shader, depth_frag_shader);

    glGenFramebuffers(1, &depthMapFBO);

    glGenTextures(1, &depthMap);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT,
                 shadowFramebufferSize.x, shadowFramebufferSize.y, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    float borderColor[] = {1.0f, 1.0f, 1.0f, 1.0f};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);

    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glGenVertexArrays(1, &quadVAO);
    glGenBuffers(1, &quadVBO);
    glBindVertexArray(quadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3*sizeof(float)));
    glBindVertexArray(0);

    pbrSolidShader->use();
    pbrSolidShader->setInt("depthMap", 8);

    /*
    glUniformBlockBinding(pbrShader->program, 0, 0);
    glUniformBlockBinding(pbrShader->program, 1, 1);
    glUniformBlockBinding(pbrShader->program, 2, 2);

    glGenBuffers(1, &dirLightUBO);
    glGenBuffers(1, &pointLightUBO);
    glGenBuffers(1, &spotLightUBO);

    glBindBuffer(GL_UNIFORM_BUFFER, dirLightUBO);
    glBufferData(GL_UNIFORM_BUFFER, sizeof(PBRDirLight), nullptr, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_UNIFORM_BUFFER, pointLightUBO);
    glBufferData(GL_UNIFORM_BUFFER, NUM_PBR_POINT_LIGHTS * sizeof(PBRPointLight), nullptr, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_UNIFORM_BUFFER, spotLightUBO);
    glBufferData(GL_UNIFORM_BUFFER, NUM_PBR_SPOT_LIGHTS * sizeof(PBRSpotLight), nullptr, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_UNIFORM_BUFFER, 0);

    glBindBufferBase(GL_UNIFORM_BUFFER, 0, dirLightUBO);
    glBindBufferBase(GL_UNIFORM_BUFFER, 1, pointLightUBO);
    glBindBufferBase(GL_UNIFORM_BUFFER, 2, spotLightUBO);
     */
}

void PBRenderer::render(bool shadows) {
    const unsigned int SCREEN_WIDTH = ImGui::GetIO().DisplaySize.x;
    const unsigned int SCREEN_HEIGHT = ImGui::GetIO().DisplaySize.y;
    float near_plane = 0.1f;
    float far_plane = 1000.0f;

    glm::mat4 dirLightProjection = glm::ortho(dirLightProjVolume.min.x, dirLightProjVolume.max.x,
                                              dirLightProjVolume.min.y, dirLightProjVolume.max.y,
                                              dirLightProjVolume.min.z, dirLightProjVolume.max.z);

    glm::vec3 dirLightPos = -glm::normalize(dirLight.direction) * dirLightProjVolume.max.z * 0.5f;
    glm::mat4 dirLightView = glm::lookAt(dirLightPos, glm::vec3(0.0f), {0.0f, 1.0f, 0.0f});
    glm::mat4 dirLightSpaceMatrix = dirLightProjection * dirLightView;

    depthShader->use();
    depthShader->setMat4("dirLightSpaceMatrix", dirLightSpaceMatrix);

    GLint origViewport[4];
    glGetIntegerv(GL_VIEWPORT, origViewport);

    glViewport(0, 0, shadowFramebufferSize.x, shadowFramebufferSize.y);
    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    {
        glClear(GL_DEPTH_BUFFER_BIT);
        if (shadows) {
            glCullFace(GL_FRONT);
            renderPass(depthShader);
            glCullFace(GL_BACK);
        }
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glViewport(origViewport[0], origViewport[1], origViewport[2], origViewport[3]);

    pbrSolidShader->use();

    if (camera) {
        pbrSolidShader->setCamera(camera);
    }
    else {
        std::cerr << "Error in PBRenderer: camera not set" << std::endl;
    }

    /*
    glBindBuffer(GL_UNIFORM_BUFFER, dirLightUBO);
    glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(PBRDirLight), &dirLight);

    glBindBuffer(GL_UNIFORM_BUFFER, pointLightUBO);
    glBufferSubData(GL_UNIFORM_BUFFER, 0, NUM_PBR_POINT_LIGHTS * sizeof(PBRPointLight), &pointLights);

    glBindBuffer(GL_UNIFORM_BUFFER, spotLightUBO);
    glBufferSubData(GL_UNIFORM_BUFFER, 0, NUM_PBR_SPOT_LIGHTS * sizeof(PBRSpotLight), &spotLights);
     */

    pbrSolidShader->setBool("dirLight.enabled", dirLight.enabled);
    if (dirLight.enabled) {
        pbrSolidShader->setVec3("dirLight.direction", dirLight.direction);
        pbrSolidShader->setVec3("dirLight.color", dirLight.color);
    }

    for (int i = 0; i < NUM_PBR_POINT_LIGHTS; i++) {
        std::string lname = std::string("pointLights[") + std::to_string(i) + "]";
        pbrSolidShader->setBool((lname + ".enabled").c_str(), pointLights[i].enabled);
        if (pointLights[i].enabled) {
            pbrSolidShader->setVec3((lname + ".position").c_str(), pointLights[i].position);
            pbrSolidShader->setVec3((lname + ".color").c_str(), pointLights[i].color);
        }
    }

    for (int i = 0; i < NUM_PBR_SPOT_LIGHTS; i++) {
        std::string lname = std::string("spotLights[") + std::to_string(i) + "]";
        pbrSolidShader->setBool((lname + ".enabled").c_str(), spotLights[i].enabled);
        if (spotLights[i].enabled) {
            pbrSolidShader->setVec3((lname + ".position").c_str(), spotLights[i].position);
            pbrSolidShader->setVec3((lname + ".direction").c_str(), spotLights[i].direction);
            pbrSolidShader->setVec3((lname + ".color").c_str(), spotLights[i].color);
        }
    }

    pbrSolidShader->setMat4("dirLightSpaceMatrix", dirLightSpaceMatrix);

    glActiveTexture(GL_TEXTURE8);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    pbrSolidShader->setInt("shadowMap", 8);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    renderPass(pbrSolidShader);
    glDisable(GL_BLEND);

    renderCommands.clear();
}

void PBRenderer::renderImGui() {
    ImGui::SetNextWindowPos(ImVec2(1620, 30), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(250, 250), ImGuiCond_FirstUseEver);

    ImGui::Begin("PBRenderer Settings");

    if (ImGui::CollapsingHeader("Directional Light Settings")) {
        ImGui::Checkbox("Enabled", (bool*) &dirLight.enabled);
        ImGui::DragFloat3("Direction", (float *) &dirLight.direction, 0.01f, -5.0f, 5.0f);
        ImGui::DragFloat3("Color", (float *) &dirLight.color, 0.01f, 0.0f, 1.0f);
    }

    if (ImGui::CollapsingHeader("Point Light Settings")) {
        for (int i = 0; i < pointLights.size(); i++) {
            auto label = fmt::format("PointLight {}", i + 1);
            if (ImGui::TreeNode(label.c_str())) {
                ImGui::PushID(i);
                ImGui::Checkbox("Enabled##pointlight", (bool*) &pointLights[i].enabled);
                ImGui::DragFloat3("Position##pointlight", (float*) &pointLights[i].position, 0.01f);
                ImGui::DragFloat3("Color##pointlight", (float*) &pointLights[i].color, 0.01f);
                ImGui::TreePop();
                ImGui::PopID();
            }
        }
    }

    if (ImGui::CollapsingHeader("Spot Light Settings")) {
        for (int i = 0; i < spotLights.size(); i++) {
            auto label = fmt::format("SpotLight {}", i + 1);
            if (ImGui::TreeNode(label.c_str())) {
                ImGui::PushID(i);
                ImGui::Checkbox("Enabled##spotlight", (bool*) &spotLights[i].enabled);
                ImGui::DragFloat3("Position##spotlight", (float*) &spotLights[i].position, 0.01f);
                ImGui::DragFloat3("Color##spotlight", (float*) &spotLights[i].color, 0.01f);
                ImGui::DragFloat("Cutoff##spotlight", &spotLights[i].cutOff, 0.01f, 0.0f, 1.0f);
                ImGui::DragFloat("OuterCutoff##spotlight", &spotLights[i].outerCutOff, 0.01f, 0.0f, 1.0f);
                ImGui::TreePop();
                ImGui::PopID();
            }
        }
    }

    ImGui::End();
}

void PBRenderer::renderPass(Ref<Shader> shader) {
    shader->use();
    for (const auto& command : renderCommands) {
        shader->setMat4("model", command.modelMatrix);
        shader->setPBRMaterial(*command.material);

        glBindVertexArray(command.mesh->vao);
        if (command.mesh->indices.empty()) {
            glDrawArrays(GL_TRIANGLES, 0, command.mesh->vertices.size());
        }
        else {
            glDrawElements(GL_TRIANGLES, command.mesh->indices.size(), GL_UNSIGNED_INT, 0);
        }
        glBindVertexArray(0);
    }
}


