//
// Created by lasagnaphil on 19. 8. 14..
//

#ifndef GENGINE_CAMERA_H
#define GENGINE_CAMERA_H

#include "Ray.h"

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/mat4x4.hpp>
#include <imgui.h>
#include <SDL2/SDL.h>

struct IntRect {
    int x, y, width, height;
};

struct Camera {
    Camera() = default;
    virtual ~Camera() = default;

    virtual void update(float dt) = 0;
    virtual void updateCameraVectors() = 0;
    virtual void renderImGui() = 0;
    virtual void processInput(SDL_Event& ev) = 0;

    Ray screenPointToRay(glm::vec2 mousePos) const {
        auto io = ImGui::GetIO();
        auto screenSize = glm::vec2 {io.DisplaySize.x, io.DisplaySize.y};

        // Thanks to http://antongerdelan.net/opengl/raycasting.html
        auto homogeneousCoord = glm::vec4 {
                2.0f * mousePos.x / screenSize.x - 1.0f,
                1.0f - 2.0f * mousePos.y / screenSize.y,
                -1.0f,
                1.0f};
        auto cameraCoord = glm::inverse(getPerspectiveMatrix()) * homogeneousCoord;
        cameraCoord.z = -1.0; cameraCoord.w = 0.0;
        auto worldCoord = glm::vec3(glm::inverse(getViewMatrix()) * cameraCoord);
        worldCoord = glm::normalize(worldCoord);

        return Ray {getGlobalPosition(), worldCoord};
    }

    virtual glm::mat4 getPerspectiveMatrix() const = 0;
    virtual glm::mat4 getViewMatrix() const = 0;
    virtual glm::vec3 getPosition() const = 0;
    virtual glm::vec3 getGlobalPosition() const = 0;
};

#endif //GENGINE_CAMERA_H
