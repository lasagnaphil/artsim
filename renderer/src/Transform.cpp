//
// Created by lasagnaphil on 19. 3. 7.
//

#include "gengine/Transform.h"
#include "gengine/Arena.h"

void Transform::addChildToParent(Ref<Transform> child, Ref<Transform> parent) {
    child->parent = parent;
    parent->children.push_back(child);
}

void Transform::setDirtyFlag(bool dirtyFlag) {
    this->dirtyFlag = dirtyFlag;
    if (dirtyFlag) {
        for (Ref<Transform> child : children) {
            child->setDirtyFlag(dirtyFlag);
        }
    }
}

glm::vec3 Transform::getGlobalScale() const {
    if (parent) {
        return parent->getGlobalScale() * scale;
    }
    else {
        return scale;
    }
}

void Transform::updateRecursive(const glm::mat4 &curTransform) {
    if (!getEnabled()) return;
    updateTransform();
    worldTransform = curTransform * localTransform;
    for (Ref<Transform> child : children) {
        child->updateRecursive(worldTransform);
    }
}

void Transform::setGlobalPosition(const glm::vec3 &position) {
    this->position = glm::inverse(parent->getWorldTransform()) * glm::vec4(position, 1.0f);
    setDirtyFlag(true);
}

void Transform::moveGlobal(const glm::vec3 &offset) {
    setPosition(position + glm::vec3(
            glm::inverse(parent->getWorldTransform()) * glm::vec4(offset, 0.0f)));
}

