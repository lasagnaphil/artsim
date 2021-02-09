//
// Created by lasagnaphil on 5/1/18.
//

#ifndef GENGINE_TRANSFORM_H
#define GENGINE_TRANSFORM_H

#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>

#include "gengine/Arena.h"

class Transform {
public:
    static void addChildToParent(Ref<Transform> child, Ref<Transform> parent);

    Transform(glm::vec3 position = {0.f, 0.f, 0.f},
              glm::quat rotation = {1.f, 0.f, 0.f, 0.f},
              glm::vec3 scale = {1.f, 1.f, 1.f}) :
        position(position), rotation(rotation), scale(scale) {

    }

    glm::mat4 toMat4() {
        glm::mat4 mat;
        mat = glm::scale(glm::mat4(1.0f), scale);
        mat = glm::mat4_cast(rotation) * mat;
        mat = glm::translate(glm::mat4(1.0f), position) * mat;
        return mat;
    }

    void setDirtyFlag(bool dirtyFlag);

    void updateTransform() {
        if (dirtyFlag) {
            localTransform = toMat4();
            dirtyFlag = false;
        }
    }

    void update() {
        updateRecursive(glm::mat4(1.0f));
    }

    void updateRecursive(const glm::mat4& curTransform);

    bool getEnabled() const {
        return enabled;
    }

    Ref<Transform> getParent() const {
        return parent;
    }

    std::vector<Ref<Transform>> getChildren() const {
        return children;
    }

    glm::vec3 getPosition() const {
        return position;
    }

    void setPosition(const glm::vec3 &position) {
        this->position = position;
        setDirtyFlag(true);
    }

    void setGlobalPosition(const glm::vec3 &position);

    void move(const glm::vec3& offset) {
        setPosition(position + offset);
    }

    void moveGlobal(const glm::vec3& offset);

    glm::vec3 getScale() const {
        return scale;
    }

    void setScale(const glm::vec3 &scale) {
        this->scale = scale;
        setDirtyFlag(true);
    }

    glm::quat getRotation() const {
        return rotation;
    }

    void setRotation(const glm::quat& rotation) {
        this->rotation = rotation;
        setDirtyFlag(true);
    }

    void setRotationEuler(const glm::vec3 rotation) {
        this->rotation = glm::quat(rotation);
        setDirtyFlag(true);
    }

    void rotate(float angleRadians, const glm::vec3& axis) {
        rotate(glm::angleAxis(angleRadians, axis));
    }

    void rotate(const glm::quat& rotation) {
        this->rotation = rotation * this->rotation;
        setDirtyFlag(true);
    }

    void rotateLocal(const glm::quat& rotation) {
        this->rotation = this->rotation * rotation;
        setDirtyFlag(true);
    }

    void pointAt(const glm::vec3& direction) {
        glm::vec3 dir = glm::normalize(direction);
        float scale = 1 / glm::length(glm::vec3(0.0f, 0.0f, 1.0f) + dir);
        float quatR = scale * (1 + dir.z);
        glm::vec3 quatI = scale * glm::cross({0.0f, 0.0f, 1.0f}, dir);
        this->rotation = glm::quat(quatR, quatI.x, quatI.y, quatI.z);
        setDirtyFlag(true);
    }

    glm::vec3 getGlobalPosition() const {
        return glm::vec3(worldTransform * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
    }

    glm::vec3 getGlobalFrontVec() const {
        return glm::normalize(glm::vec3(worldTransform * glm::vec4(0.0f, 0.0f, 1.0f, 0.0f)));
    }

    glm::vec3 getGlobalUpVec() const {
        return glm::normalize(glm::vec3(worldTransform * glm::vec4(0.0f, 1.0f, 0.0f, 0.0f)));
    }

    glm::vec3 getGlobalRightVec() const {
        return glm::normalize(glm::vec3(worldTransform * glm::vec4(1.0f, 0.0f, 0.0f, 0.0f)));
    }

    glm::vec3 getFrontVec() const {
        return glm::normalize(glm::vec3(localTransform * glm::vec4(0.0f, 0.0f, 1.0f, 0.0f)));
        // return glm::rotate(rotation, glm::vec3(0.0f, 0.0f, 1.0f));
    }

    glm::vec3 getUpVec() const {
        return glm::normalize(glm::vec3(localTransform * glm::vec4(0.0f, 1.0f, 0.0f, 0.0f)));
        // return glm::rotate(rotation, glm::vec3(0.0f, 1.0f, 0.0f));
    }

    glm::vec3 getRightVec() const {
        return glm::normalize(glm::vec3(localTransform * glm::vec4(1.0f, 0.0f, 0.0f, 0.0f)));
        // return glm::rotate(rotation, glm::vec3(1.0f, 0.0f, 0.0f));
    }

    glm::mat4 getLocalTransform() const {
        return localTransform;
    }

    glm::mat4 getWorldTransform() const {
        return worldTransform;
    }

    glm::vec3 getGlobalScale() const;

    void setGlobalScale(const glm::vec3& newScale) {
        auto globalScale = getGlobalScale();
        scale.x *= newScale.x / globalScale.x;
        scale.y *= newScale.y / globalScale.y;
        scale.z *= newScale.z / globalScale.z;

        setDirtyFlag(true);
    }

private:
    glm::vec3 position;
    glm::quat rotation;
    glm::vec3 scale;

    glm::mat4 localTransform {1.0f};
    glm::mat4 worldTransform {1.0f};

    Ref<Transform> parent = {};
    std::vector<Ref<Transform>> children;

    bool enabled = true;
    bool dirtyFlag = true;
};

#endif //GENGINE_TRANSFORM_H
