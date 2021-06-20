//
// Created by lasagnaphil on 19. 6. 19.
//

#ifndef PHYSICS_BENCHMARKS_POSE_H
#define PHYSICS_BENCHMARKS_POSE_H

#include <artsim/math/common.h>
#include <artsim/math/se3.h>
#include <artsim/math/euler.h>

#include <vector>
#include <glm/vec3.hpp>

#ifndef GLM_ENABLE_EXPERIMENTAL
#define GLM_ENABLE_EXPERIMENTAL
#endif

#include <glm/gtx/euler_angles.hpp>
#include <glm/ext/quaternion_common.hpp>
#include <glm/ext/quaternion_float.hpp>
#include <glm/ext/quaternion_trigonometric.hpp>

/*
 * Represents a pose of an aritculated figure.
 * In mathematical terms, P = (v0, q1, q2, ..., qn).
 */

namespace glmx {
    struct pose;

    struct pose_view {
        float* v_ptr = nullptr;
        float* q_ptr = nullptr;
        size_t _size = 0;

        pose_view() = default;
        pose_view(float* v_ptr, float* q_ptr, size_t size) :
            v_ptr(v_ptr), q_ptr(q_ptr), _size(size) {}
        pose_view(glm::vec3* v_ptr, glm::quat* q_ptr, size_t size) :
                v_ptr((float*)v_ptr), q_ptr((float*)q_ptr), _size(size) {}
        inline pose_view(pose& pose);

        size_t size() const {
            return _size;
        }
        glm::vec3& v() {
            return *((glm::vec3*)v_ptr);
        }

        const glm::vec3& v() const {
            return *((glm::vec3*)v_ptr);
        }

        glm::quat& q(size_t i) {
            assert(i < _size);
            return ((glm::quat*)q_ptr)[i];
        }

        const glm::quat& q(size_t i) const {
            assert(i < _size);
            return ((glm::quat*)q_ptr)[i];
        }

        glmx::quat_transform getRoot() {
            return {v(), q(0)};
        }
    };

    struct const_pose_view {
        const float* v_ptr = nullptr;
        const float* q_ptr = nullptr;
        size_t _size = 0;

        const_pose_view(const float* v_ptr, const float* q_ptr, size_t size) :
                v_ptr(v_ptr), q_ptr(q_ptr), _size(size) {}
        const_pose_view(const glm::vec3* v_ptr, const glm::quat* q_ptr, size_t size) :
                v_ptr((float*)v_ptr), q_ptr((float*)q_ptr), _size(size) {}

        inline const_pose_view(pose_view view) :
                v_ptr(view.v_ptr), q_ptr(view.q_ptr), _size(view._size) {}

        inline const_pose_view(const pose& pose);

        size_t size() const {
            return _size;
        }
        glm::vec3& v() {
            return *((glm::vec3*)v_ptr);
        }

        const glm::vec3& v() const {
            return *((glm::vec3*)v_ptr);
        }

        glm::quat& q(size_t i) {
            assert(i < _size);
            return ((glm::quat*)q_ptr)[i];
        }

        const glm::quat& q(size_t i) const {
            assert(i < _size);
            return ((glm::quat*)q_ptr)[i];
        }

        glmx::transform getRoot() {
            return {v(), glm::mat3_cast(q(0))};
        }
    };


    struct pose {
        glm::vec3 _v;
        std::vector<glm::quat> _q;

        pose() = default;

        pose(glm::vec3 rootPos, std::vector<glm::quat> jointRot) :
                _v(rootPos), _q(std::move(jointRot)) {}

        explicit pose(pose_view view) {
            _v = view.v();
            glm::quat* q_ptr = (glm::quat*)view.q_ptr;
            _q = std::vector<glm::quat>(q_ptr, q_ptr + view.size());
        }
        explicit pose(const_pose_view view) {
            _v = view.v();
            glm::quat* q_ptr = (glm::quat*)view.q_ptr;
            _q = std::vector<glm::quat>(q_ptr, q_ptr + view.size());
        }

        static pose empty(std::size_t n) {
            auto q = std::vector<glm::quat>(n, glm::identity<glm::quat>());
            return {glm::vec3(0.0f), q};
        }

        glm::vec3& v() { return _v; }

        const glm::vec3& v() const { return _v; }

        glm::quat& q(size_t i) {
            assert(i < _q.size());
            return _q[i];
        }

        const glm::quat& q(size_t i) const {
            assert(i < _q.size());
            return _q[i];
        }

        size_t size() const { return _q.size(); }

        pose_view getView() {
            return pose_view(&_v, _q.data(), _q.size());
        }
        const_pose_view getConstView() const {
            return const_pose_view(&_v, _q.data(), _q.size());
        }

        glmx::quat_transform getRoot() {
            return {_v, _q[0]};
        }
    };

    pose_view::pose_view(pose& pose) : pose_view(&pose._v, pose._q.data(), pose._q.size()) {}
    const_pose_view::const_pose_view(const pose& pose) : const_pose_view(&pose._v, pose._q.data(), pose._q.size()) {}

    inline void slerp(pose_view p1, pose_view p2, float alpha, pose_view p3) {
        assert(p1.size() == p2.size() && p1.size() == p3.size());
        p3.v() = (1 - alpha) * p1.v() + alpha * p2.v();
        for (size_t i = 0; i < p3.size(); i++) {
            p3.q(i) = glm::slerp(p1.q(i), p2.q(i), alpha);
        }
    }

    inline void lerp(pose_view p1, pose_view p2, float alpha, pose_view p3) {
        assert(p1.size() == p2.size() && p1.size() == p3.size());
        p3.v() = (1 - alpha) * p1.v() + alpha * p2.v();
        for (size_t i = 0; i < p3.size(); i++) {
            p3.q(i) = glm::normalize(glm::lerp(p1.q(i), p2.q(i), alpha));
        }
    }

    inline void slerp(const pose& p1, const pose& p2, float alpha, pose& p3) {
        p3._q.resize(p1.size());
        assert(p1.size() == p2.size());
        p3._v = (1 - alpha) * p1._v + alpha * p2._v;
        for (size_t i = 0; i < p3.size(); i++) {
            p3._q[i] = glm::slerp(p1._q[i], p2._q[i], alpha);
        }
    }

    inline void lerp(const pose& p1, const pose& p2, float alpha, pose& p3) {
        p3._q.resize(p1.size());
        assert(p1.size() == p2.size());
        p3._v = (1 - alpha) * p1._v + alpha * p2._v;
        for (size_t i = 0; i < p3.size(); i++) {
            p3._q[i] = glm::normalize(glm::lerp(p1._q[i], p2._q[i], alpha));
        }
    }

    inline std::vector<glm::vec3> pose_differentiate(const_pose_view p1, const_pose_view p2, float dt) {
        std::vector<glm::vec3> diff(p1.size() + 1);
        diff[0] = p2.v() - p1.v();
        for (int i = 0; i < p1.size(); i++) {
            diff[i+1] = glmx::log(glm::inverse(p1.q(i)) * p2.q(i)) / dt;
        }
        return diff;
    }

    inline void pose_differentiate(const_pose_view p1, const_pose_view p2, float dt, glm::vec3* diff) {
        diff[0] = p2.v() - p1.v();
        for (int i = 0; i < p1.size(); i++) {
            diff[i+1] = glmx::log(glm::inverse(p1.q(i)) * p2.q(i)) / dt;
        }
    }
}

#endif //PHYSICS_BENCHMARKS_POSE_H
