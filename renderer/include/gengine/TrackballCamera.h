//
// Created by lasagnaphil on 19. 8. 14..
//

#ifndef GENGINE_TRACKBALLCAMERA_H
#define GENGINE_TRACKBALLCAMERA_H

#include "gengine/Shader.h"
#include "gengine/Ray.h"
#include "gengine/Transform.h"
#include "gengine/Camera.h"
#include <imgui.h>
#include <SDL2/SDL_events.h>

class TrackballCamera : public Camera {
public:
    TrackballCamera() = default;
    explicit TrackballCamera(Ref<Transform> parent);

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

    Ref<Transform> transform;
    Ref<Transform> trackballFocus;

    float fov = 90.0f;
    float pnear = 0.1f;
    float pfar = 1000.0f;

    float radius = 300.0f;
    float distance = 10.0f;
    float translationSpeed = 10.0f;
    // float movementSpeed = 10.0f;
    // float mouseSensitivity = 0.1f;

    // debug
    float theta = 0.0f;

private:
    glm::vec3 calcMouseVec(glm::vec2 mousePos);

    IntRect viewport;
    bool constrainPitch = true;

    // UI state
    bool enableZoom = false;
};

#endif //GENGINE_TRACKBALLCAMERA_H
