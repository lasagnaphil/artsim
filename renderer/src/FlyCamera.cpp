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

    if (mode == ViewMode::Perspective) {
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
    else {
        // Keyboard movement
        float velocity = movementSpeed * dt;
        if (inputMgr->isMousePressed(SDL_SCANCODE_LSHIFT)) {
            velocity = velocity * 0.1f;
        }
        if (inputMgr->isKeyPressed(SDL_SCANCODE_W)) {
            transform->move(transform->getUpVec() * velocity);
        }
        else if (inputMgr->isKeyPressed(SDL_SCANCODE_S)) {
            transform->move(-transform->getUpVec() * velocity);
        }
        if (inputMgr->isKeyPressed(SDL_SCANCODE_A)) {
            transform->move(-transform->getRightVec() * velocity);
        }
        else if (inputMgr->isKeyPressed(SDL_SCANCODE_D)) {
            transform->move(transform->getRightVec() * velocity);
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
    auto trans = transform.get();
    if (ev.type == SDL_MOUSEWHEEL) {
        if (mode == ViewMode::Perspective) {
            if (enableMiddleScroll) {
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
        else {
            float increment = 0.25f * ev.wheel.y;
            orthoZoom = glm::clamp(orthoZoom + increment, 0.01f, 10.0f);
        }
    }
}

// TODO: cache the perspective and view matrices, to reduce repeating the same computation
glm::mat4 FlyCamera::getPerspectiveMatrix() const {
    GLint gl_viewport[4];
    glGetIntegerv(GL_VIEWPORT, gl_viewport);
    float aspectRatio = (float)gl_viewport[2] / (float)gl_viewport[3];
    glm::vec3 pos = transform->getGlobalPosition();
    switch (mode) {
        case ViewMode::Perspective: {
            return glm::perspective(
                    glm::radians(fov),
                    aspectRatio,
                    near,
                    far
            );
        }
        case ViewMode::Projective_PlusX: {
            return glm::ortho(
                    pos.z + 0.5f * aspectRatio / orthoZoom,
                    pos.z - 0.5f * aspectRatio / orthoZoom,
                    pos.y - 0.5f / orthoZoom,
                    pos.y + 0.5f / orthoZoom,
                    near,
                    far);
        }
        case ViewMode::Projective_MinusX: {
            return glm::ortho(
                    pos.z - 0.5f * aspectRatio / orthoZoom,
                    pos.z + 0.5f * aspectRatio / orthoZoom,
                    pos.y - 0.5f / orthoZoom,
                    pos.y + 0.5f / orthoZoom,
                    near,
                    far);
        }
        case ViewMode::Projective_PlusY: {
            return glm::ortho(
                    pos.x + 0.5f * aspectRatio / orthoZoom,
                    pos.x - 0.5f * aspectRatio / orthoZoom,
                    pos.z - 0.5f / orthoZoom,
                    pos.z + 0.5f / orthoZoom,
                    near,
                    far);
        }
        case ViewMode::Projective_MinusY: {
            return glm::ortho(
                    pos.x - 0.5f * aspectRatio / orthoZoom,
                    pos.x + 0.5f * aspectRatio / orthoZoom,
                    pos.z - 0.5f / orthoZoom,
                    pos.z + 0.5f / orthoZoom,
                    near,
                    far);
        }
        case ViewMode::Projective_PlusZ: {
            return glm::ortho(
                    pos.x + 0.5f * aspectRatio / orthoZoom,
                    pos.x - 0.5f * aspectRatio / orthoZoom,
                    pos.y - 0.5f / orthoZoom,
                    pos.y + 0.5f / orthoZoom,
                    near,
                    far);
        }
        case ViewMode::Projective_MinusZ: {
            return glm::ortho(
                    pos.x - 0.5f * aspectRatio / orthoZoom,
                    pos.x + 0.5f * aspectRatio / orthoZoom,
                    pos.y - 0.5f / orthoZoom,
                    pos.y + 0.5f / orthoZoom,
                    near,
                    far);
        }
        default:
            return glm::mat4();
    }

}

glm::mat4 FlyCamera::getViewMatrix() const {
    return glm::lookAt(
            transform->getGlobalPosition(),
            transform->getGlobalPosition() - transform->getGlobalFrontVec(),
            transform->getGlobalUpVec());
}

glm::vec2 FlyCamera::worldPointToScreen(const glm::vec3& pos) {
    glm::vec4 res = getPerspectiveMatrix() * getViewMatrix() * glm::vec4(pos, 1.f);
    res.x /= res.w;
    res.y /= res.w;
    res.z /= res.w;
    GLint gl_viewport[4];
    glGetIntegerv(GL_VIEWPORT, gl_viewport);
    glm::vec2 screenPos = {0.5f * (1 + res.x) * gl_viewport[2], 0.5f * (1 - res.y) * gl_viewport[3]};
    return screenPos;
}

void FlyCamera::setViewMode(FlyCamera::ViewMode _mode) {
    mode = _mode;
    switch(mode) {
        case ViewMode::Perspective: {
            if (pitch > 89.0f) pitch = 89.0f;
            if (pitch < -89.0f) pitch = -89.0f;
        } break;
        case ViewMode::Projective_PlusX: yaw = 90; pitch = 0; break;
        case ViewMode::Projective_MinusX: yaw = -90; pitch = 0; break;
        case ViewMode::Projective_PlusY: yaw = 0; pitch = 90; break;
        case ViewMode::Projective_MinusY: yaw = 0; pitch = -90; break;
        case ViewMode::Projective_PlusZ: yaw = 180; pitch = 0; break;
        case ViewMode::Projective_MinusZ: yaw = 0; pitch = 0; break;
    }
    updateCameraVectors();
}


