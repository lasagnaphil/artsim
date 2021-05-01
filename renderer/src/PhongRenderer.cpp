//
// Created by lasagnaphil on 19. 9. 28..
//

#include "gengine/PhongRenderer.h"
#include "fmt/core.h"

#include "shaders/depth.vert.h"
#include "shaders/depth.frag.h"
#include "shaders/depth_debug.vert.h"
#include "shaders/depth_debug.frag.h"
#include "shaders/phong.vert.h"
#include "shaders/phong.frag.h"

PhongRenderer::PhongRenderer(Rect3f projVolume, glm::ivec2 shadowFBSize, Camera *camera) :

        camera(camera), shadowFramebufferSize(shadowFBSize),
        dirLightProjVolume(projVolume) {

    dirLight.enabled = true;
    dirLight.direction = glm::normalize(glm::vec3 {2.0f, -3.0f, 2.0f});
    dirLight.ambient = {0.3f, 0.3f, 0.3f, 0.3f};
    dirLight.diffuse = {1.0f, 1.0f, 1.0f, 1.0f};
    dirLight.specular = {1.0f, 1.0f, 1.0f, 1.0f};
    dirLight.intensity = 0.5f;
}

void PhongRenderer::init() {
    if (camera == nullptr) {
        fmt::print(stderr, "Camera not attached to PhongRenderer!\n");
        exit(EXIT_FAILURE);
    }

    depthShader->compileFromString("depth", depth_vert_shader, depth_frag_shader);

    debugDepthShader->compileFromString("depth_debug", depth_debug_vert_shader, depth_debug_frag_shader);
    phongShader->compileFromString("phong", phong_vert_shader, phong_frag_shader);

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

    phongShader->use();
    phongShader->setInt("diffuseTexture", 0);
    phongShader->setInt("shadowMap", 1);
    debugDepthShader->use();
    debugDepthShader->setInt("depthMap", 0);
}

void PhongRenderer::render() {
    // renderPass();

    const unsigned int SCREEN_WIDTH = ImGui::GetIO().DisplaySize.x;
    const unsigned int SCREEN_HEIGHT = ImGui::GetIO().DisplaySize.y;
    float near_plane = 0.1f;
    float far_plane = 1000.f;

    glm::mat4 dirLightProjection = glm::ortho(dirLightProjVolume.left, dirLightProjVolume.right,
                                              dirLightProjVolume.bottom, dirLightProjVolume.top, dirLightProjVolume.zNear, dirLightProjVolume.zFar);

    glm::vec3 dirLightPos = -glm::normalize(dirLight.direction) * dirLightProjVolume.zFar * 0.5f;
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
        glCullFace(GL_FRONT);
        renderPass(depthShader);
        glCullFace(GL_BACK);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glViewport(origViewport[0], origViewport[1], origViewport[2], origViewport[3]);

    phongShader->use();

    if (camera) {
        phongShader->setCamera(camera);
    }
    else {
        std::cerr << "Error in PhongRenderer: camera not set" << std::endl;
    }

    phongShader->setBool("dirLight.enabled", dirLight.enabled);
    phongShader->setVec3("dirLight.direction", dirLight.direction);
    phongShader->setVec4("dirLight.ambient", dirLight.ambient);
    phongShader->setVec4("dirLight.diffuse", dirLight.diffuse);
    phongShader->setVec4("dirLight.specular", dirLight.specular);
    phongShader->setFloat("dirLight.intensity", dirLight.intensity);
    phongShader->setMat4("dirLightSpaceMatrix", dirLightSpaceMatrix);

    glActiveTexture(GL_TEXTURE8);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    phongShader->setInt("shadowMap", 8);

    renderPass(phongShader);

    if (viewDepthBufferDebug) {
        debugDepthShader->use();
        debugDepthShader->setFloat("near_plane", near_plane);
        debugDepthShader->setFloat("far_plane", far_plane);
        glActiveTexture(GL_TEXTURE8);
        glBindTexture(GL_TEXTURE_2D, depthMap);
        debugDepthShader->setInt("depthMap", 8);
        renderDepthMapDebug();
    }

    renderCommands.clear();
}

void PhongRenderer::renderImGui() {

    ImGui::SetNextWindowPos(ImVec2(1620, 30), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(250, 250), ImGuiCond_FirstUseEver);

    ImGui::Begin("PhongRenderer Settings");

    if (ImGui::TreeNode("Directional Light Settings")) {
        ImGui::Checkbox("Enabled", &dirLight.enabled);

        ImGui::SliderFloat3("Direction", (float *) &dirLight.direction, -5.0f, 5.0f);
        ImGui::SliderFloat3("Ambient", (float *) &dirLight.ambient, 0.0f, 1.0f);
        ImGui::SliderFloat3("Diffuse", (float *) &dirLight.diffuse, 0.0f, 1.0f);
        ImGui::SliderFloat3("Specular", (float *) &dirLight.specular, 0.0f, 1.0f);
        ImGui::SliderFloat("Intensity", (float *) &dirLight.intensity, 0.0f, 1.0f);

        ImGui::InputFloat2("Left/Right", (float *) &dirLightProjVolume.left);
        ImGui::InputFloat2("Bottom/Top", (float *) &dirLightProjVolume.bottom);
        ImGui::InputFloat2("zNear/zFar", (float *) &dirLightProjVolume.zNear);

        ImGui::TreePop();
    }

    ImGui::End();
}

void PhongRenderer::renderPass(Ref<Shader> shader) {
    shader->use();
    for (const PhongRenderCommand& command : renderCommands) {
        shader->setMat4("model", command.modelMatrix);
        shader->setPhongMaterial(*command.material);

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

void PhongRenderer::renderDepthMapDebug() {
    static GLuint quadVAO = 0;
    static GLuint quadVBO;
    if (quadVAO == 0) {
        float quadVertices[] = {
                // positions        // texture Coords
                -1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
                -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
                1.0f,  1.0f, 0.0f, 1.0f, 1.0f,
                1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
        };
        // setup plane VAO
        glGenVertexArrays(1, &quadVAO);
        glGenBuffers(1, &quadVBO);
        glBindVertexArray(quadVAO);
        glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    }
    glBindVertexArray(quadVAO);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glBindVertexArray(0);
}
