//
// Created by lasagnaphil on 4/7/18.
//

#ifndef GENGINE_FLYCAMERA_H
#define GENGINE_FLYCAMERA_H

#include "gengine/Shader.h"
#include "gengine/Ray.h"
#include "gengine/Transform.h"
#include "gengine/Camera.h"

#include <imgui.h>
#include <SDL2/SDL_events.h>

class FlyCamera : public Camera {
public:
    FlyCamera() = default;
    explicit FlyCamera(Ref<Transform> parent, glm::ivec2 windowSize);

    void update(float dt) override;
    void updateCameraVectors() override;
    void renderImGui() override;
    void processInput(SDL_Event& ev) override;

    glm::mat4 getGlobalTransform() const override { return transform->getWorldTransform(); }
    glm::mat4 getPerspectiveMatrix() const override;
    glm::mat4 getViewMatrix() const override;
    glm::vec3 getPosition() const override {
        return transform->getPosition();
    }
    glm::vec3 getGlobalPosition() const override {
        return transform->getGlobalPosition();
    }

    glm::vec2 worldPointToScreen(const glm::vec3& pos) const;
    glm::vec3 screenPointToWorld(const glm::vec2& screenPos, float depth) const;

    Ref<Transform> transform = {};

    float fov = 90.0f;
    float pnear = 0.1f;
    float pfar = 1000.0f;
    float movementSpeed = 10.0f;
    float mouseSensitivity = 0.1f;

    float pitch = 0.0f;
    float yaw = 0.0f;
    bool constrainPitch = true;
    bool enableZoom = false;
    bool enableMiddleScroll = false;

    bool enableHideMouse = false;

    enum class ViewMode {
        Perspective,
        Projective_PlusX, Projective_MinusX,
        Projective_PlusY, Projective_MinusY,
        Projective_PlusZ, Projective_MinusZ
    } mode = ViewMode::Perspective;

    void setViewMode(ViewMode mode);
    void setOrthoZoom(float orthoZoom) { this->orthoZoom = orthoZoom; }

private:
    float radius = 300.0f;
    float distance = 10.0f;
    float orthoZoom = 1.0f;
};


#endif //GENGINE_FLYCAMERA_H
