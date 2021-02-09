//
// Created by lasagnaphil on 19. 8. 14..
//

#include "gengine/TrackballCamera.h"
#include "gengine/InputManager.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtx/string_cast.hpp>

TrackballCamera::TrackballCamera(Ref<Transform> parent) {
    transform = Resources::make<Transform>();
    transform->setPosition(glm::vec3(0.f, 0.f, distance));

    trackballFocus = Resources::make<Transform>();
    trackballFocus->setPosition(glm::vec3(0.f, 0.f, 0.f));
    Transform::addChildToParent(transform, trackballFocus);
    Transform::addChildToParent(trackballFocus, parent);

    viewport = {0, 0, (int)ImGui::GetIO().DisplaySize.x, (int)ImGui::GetIO().DisplaySize.y};
    updateCameraVectors();
}

void TrackballCamera::update(float dt) {
    auto inputMgr = InputManager::get();

    if (inputMgr->isMousePressed(SDL_BUTTON_RIGHT)) {
        static glm::ivec2 refMousePos = {};
        static glm::ivec2 prevMousePos = {};
        static glm::ivec2 mousePos = {};

        auto displaySize = ImGui::GetIO().DisplaySize;
        glm::vec3 vref;

        if (inputMgr->isMouseEntered(SDL_BUTTON_RIGHT)) {
            refMousePos = inputMgr->getMousePos();
            mousePos = refMousePos;
            prevMousePos = refMousePos;
            auto mref = glm::vec2{refMousePos.x - displaySize.x / 2, -refMousePos.y + displaySize.y / 2};
            vref = calcMouseVec(mref);
        }

        mousePos = inputMgr->getMousePos();

        auto m1 = glm::vec2{prevMousePos.x - displaySize.x / 2, -prevMousePos.y + displaySize.y / 2};
        auto m2 = glm::vec2{mousePos.x - displaySize.x / 2, -mousePos.y + displaySize.y / 2};

        glm::vec3 v1 = calcMouseVec(m1);
        glm::vec3 v2 = calcMouseVec(m2);

        glm::quat q;
        float dotProduct = glm::dot(v2, v1);
        float lengthProduct = std::sqrt(glm::length2(v1) * glm::length2(v2));
        if (dotProduct / lengthProduct == -1) {
            q = glm::quat(1.f, 0.f, 0.f, 0.f);
        } else {
            q = glm::quat(lengthProduct + dotProduct, glm::cross(v2, v1));
            q = glm::normalize(q);
        }

        trackballFocus->rotateLocal(q);

        prevMousePos = mousePos;
    }

    auto upDir = transform->getGlobalUpVec();
    auto rightDir = transform->getGlobalRightVec();
    if (inputMgr->isKeyPressed(SDL_SCANCODE_LEFT) || inputMgr->isKeyPressed(SDL_SCANCODE_A)) {
        trackballFocus->moveGlobal(-translationSpeed * dt * rightDir);
    } else if (inputMgr->isKeyPressed(SDL_SCANCODE_RIGHT) || inputMgr->isKeyPressed(SDL_SCANCODE_D)) {
        trackballFocus->moveGlobal(translationSpeed * dt * rightDir);
    } else if (inputMgr->isKeyPressed(SDL_SCANCODE_UP) || inputMgr->isKeyPressed(SDL_SCANCODE_W)) {
        trackballFocus->moveGlobal(translationSpeed * dt * upDir);
    } else if (inputMgr->isKeyPressed(SDL_SCANCODE_DOWN) || inputMgr->isKeyPressed(SDL_SCANCODE_S)) {
        trackballFocus->moveGlobal(-translationSpeed * dt * upDir);
    }
}

void TrackballCamera::updateCameraVectors() {

}

void TrackballCamera::renderImGui() {
    auto trans = transform.get();
    auto focusTrans = trackballFocus.get();

    ImGui::Begin("Trackball");
    ImGui::Text("Position of trackball focus: %s", glm::to_string(focusTrans->getGlobalPosition()).c_str());
    ImGui::Text("Position of trackball camera: %s", glm::to_string(trans->getGlobalPosition()).c_str());
    ImGui::Text("Theta: %f", glm::degrees(theta));
    ImGui::Text("GlobalFrontVec: %s", glm::to_string(trans->getGlobalFrontVec()).c_str());
    ImGui::Text("GlobalUpVec: %s", glm::to_string(trans->getGlobalUpVec()).c_str());
    ImGui::Text("Distance: %f", distance);
    ImGui::Text("Fov: %f", fov);
    ImGui::Checkbox("Use zoom instead of dolly", &enableZoom);
    ImGui::End();
}

void TrackballCamera::processInput(SDL_Event &ev) {

}

glm::mat4 TrackballCamera::getPerspectiveMatrix() const {
    return glm::perspective(
        glm::radians(fov),
        ImGui::GetIO().DisplaySize.x / ImGui::GetIO().DisplaySize.y,
        near,
        far
    );
}

glm::mat4 TrackballCamera::getViewMatrix() const {
    return glm::lookAt(
            transform->getGlobalPosition(),
            transform->getGlobalPosition() - transform->getGlobalFrontVec(),
            transform->getGlobalUpVec());
}

glm::vec3 TrackballCamera::calcMouseVec(glm::vec2 mousePos) {
    float z;
    float mouseRadiusSq = mousePos.x * mousePos.x + mousePos.y + mousePos.y;
    float radiusSq = radius * radius;
    if (mouseRadiusSq <= 0.5f * radiusSq) {
        z = std::sqrt(radiusSq - mouseRadiusSq);
    }
    else {
        z = 0.5f * radiusSq / std::sqrt(mouseRadiusSq);
    }

    return glm::normalize(glm::vec3(mousePos.x, mousePos.y, z));
}

