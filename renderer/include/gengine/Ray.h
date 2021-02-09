//
// Created by lasagnaphil on 19. 3. 8.
//

#ifndef MOTION_EDITING_RAY_H
#define MOTION_EDITING_RAY_H

#include <optional>
#include <glm/vec3.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/constants.hpp>

struct Ray {
    glm::vec3 pos;
    glm::vec3 dir;

    Ray(glm::vec3 pos = {0.f, 0.f, 0.f}, glm::vec3 dir = {0.f, 1.f, 0.f}) :
        pos(pos), dir(glm::normalize(dir)) {}

    bool intersectWithPlane(const Ray& plane, glm::vec3& result) const {
        float denom = glm::dot(plane.dir, dir);
        if (glm::abs(denom) <= glm::epsilon<float>()) {
            return false;
        }
        float t = glm::dot(plane.dir, plane.pos - pos) / denom;
        if (t < 0.0f) {
            return false;
        }
        result = pos + t * dir;
        return true;
    }

    float distanceToPointSq(const glm::vec3& point) const {
        glm::vec3 dist = (point - pos) - glm::dot((point - pos), dir) * dir;
        return glm::dot(dist, dist);
    }

    float distanceToPoint(const glm::vec3& point) const {
        glm::vec3 dist = (point - pos) - glm::dot((point - pos), dir) * dir;
        return glm::length(dist);
    }

    float distanceToRay(const Ray& ray) const {
        glm::vec3 n = glm::cross(dir, ray.dir);
        return glm::dot(glm::normalize(n), pos - ray.pos);
        /*
        glm::vec3 c = ray.pos - pos;
        float ab = glm::dot(dir, ray.dir);
        float bc = glm::dot(ray.dir, c);
        float ca = glm::dot(c, dir);
        float aa = glm::dot(dir, dir);
        float bb = glm::dot(ray.dir, ray.dir);
        glm::vec3 dist = ray.dir - dir + ((ab*bc - ca*bb)*dir + (ab*ca - bc*aa)*ray.dir)/(aa*bb - ab*ab);
        return glm::length(dist);
         */
    }
};

#endif //MOTION_EDITING_RAY_H
