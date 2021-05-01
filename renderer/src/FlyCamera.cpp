//
// Created by lasagnaphil on 4/7/18.
//

#include <imgui.h>
#include <SDL2/SDL_events.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtc/quaternion.hpp>

#include "gengine/Arena.h"
#include "gengine/FlyCamera.h"
#include "gengine/Transform.h"
#include "gengine/InputManager.h"

FlyCamera::FlyCamera(Ref<Transform> parent, glm::ivec2 windowSize)
{
    transform = Resources::make<Transform>();
    transform->setPosition(glm::vec3(0.f, 0.f, distance));
    Transform::addChildToParent(transform, parent);

    updateCameraVectors();
}

void FlyCamera::update(float dt) {
    auto inputMgr = InputManager::get();

    bool pressStart = false;
    if (inputMgr->isMouseEntered(SDL_BUTTON_RIGHT)) {
        if (enableHideMouse) {
            SDL_SetRelativeMouseMode(SDL_TRUE);
        }
        pressStart = true;
    }
    if (inputMgr->isMousePressed(SDL_BUTTON_RIGHT)) {
        // FPS Controls
        auto mouseOffsetI = inputMgr->getRelMousePos();
        auto mouseOffset = pressStart? glm::vec2() : glm::vec2((float) mouseOffsetI.x, (float) mouseOffsetI.y);

        mouseOffset *= mouseSensitivity;

        yaw += mouseOffset.x;
        pitch += mouseOffset.y;

        if (constrainPitch) {
            if (pitch > 89.0f) pitch = 89.0f;
            if (pitch < -89.0f) pitch = -89.0f;
        }

        updateCameraVectors();

        // Keyboard movement
        float velocity = movementSpeed * dt;
        if (inputMgr->isMousePressed(SDL_SCANCODE_LSHIFT)) {
            velocity = velocity * 0.1f;
        }
        if (inputMgr->isKeyPressed(SDL_SCANCODE_W)) {
            transform->move(-transform->getFrontVec() * velocity);
        }
        else if (inputMgr->isKeyPressed(SDL_SCANCODE_S)) {
            transform->move(transform->getFrontVec() * velocity);
        }
        if (inputMgr->isKeyPressed(SDL_SCANCODE_A)) {
            transform->move(-transform->getRightVec() * velocity);
        }
        else if (inputMgr->isKeyPressed(SDL_SCANCODE_D)) {
            transform->move(transform->getRightVec() * velocity);
        }
        if (inputMgr->isKeyPressed(SDL_SCANCODE_Q)) {
            transform->move(transform->getUpVec() * velocity);
        }
        else if (inputMgr->isKeyPressed(SDL_SCANCODE_E)) {
            transform->move(-transform->getUpVec() * velocity);
        }
    }
    else if (inputMgr->isMouseExited(SDL_BUTTON_RIGHT)) {
        if (enableHideMouse) {
            SDL_SetRelativeMouseMode(SDL_FALSE);
        }
    }
}

void FlyCamera::updateCameraVectors() {
    glm::quat quatX = glm::angleAxis(-glm::radians(yaw), glm::vec3(0.0f, 1.0f, 0.0f));
    glm::quat quatY = glm::angleAxis(-glm::radians(pitch), glm::vec3(1.0f, 0.0f, 0.0f));
    transform->setRotation(glm::normalize(quatX * quatY));
}

void FlyCamera::renderImGui() {
    // TODO
}

void FlyCamera::processInput(SDL_Event& ev) {
    if (enableMiddleScroll) {
        auto trans = transform.get();
        if (ev.type == SDL_MOUSEWHEEL) {
            if (enableZoom) {
                fov += ev.wheel.y;
            } else {
                float increment = 0.25f * ev.wheel.y;
                if (distance + increment > 0.f) {
                    distance += increment;
                }
                auto curPos = trans->getPosition();
                auto nextPos = distance / glm::length(curPos) * curPos;
                trans->setPosition(nextPos);
            }
        }
    }
}

// TODO: cache the perspective and view matrices, to reduce repeating the same computation
glm::mat4 FlyCamera::getPerspectiveMatrix() const {
    GLint gl_viewport[4];
    glGetIntegerv(GL_VIEWPORT, gl_viewport);
    return glm::perspective(
            glm::radians(fov),
            (float)gl_viewport[2] / (float)gl_viewport[3],
            near,
            far
    );
}

glm::mat4 FlyCamera::getViewMatrix() const {
    return glm::lookAt(
            transform->getGlobalPosition(),
            transform->getGlobalPosition() - transform->getGlobalFrontVec(),
            transform->getGlobalUpVec());
}


