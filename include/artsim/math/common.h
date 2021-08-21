//
// Created by lasagnaphil on 20. 9. 10..
//

#ifndef ARTSIM_COMMON_H
#define ARTSIM_COMMON_H

#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/norm.hpp>

namespace glmx {
    enum Identity {
        IDENTITY
    };

    template <class T>
    inline glm::tvec3<T> log(glm::tquat<T> q) {
        constexpr T pi = glm::pi<T>();
        q = glm::normalize(q);
        T a = glm::sqrt(1 - q.w*q.w);
        if (a <= glm::epsilon<T>()) {
            return glm::tvec3<T>(0);
        }
        T theta = T(2.0) * glm::atan(a, q.w);
        if (theta > pi) {
            theta -= 2*pi;
        }
        else if (theta < -pi) {
            theta += 2*pi;
        }
        glm::tvec3<T> v = theta / a * glm::tvec3<T>(q.x, q.y, q.z);
        return v;
    }

    template <class T>
    inline glm::tvec3<T> logdiff(glm::tquat<T> q1, glm::tquat<T> q2) {
        // return glmx::log(q2 * glm::conjugate(q1));
        return glmx::log(glm::conjugate(q1) * q2);
    }

    template <class T>
    inline glm::tquat<T> exp(glm::tvec3<T> v) {
        T theta = glm::length(v);
        if (theta <= glm::epsilon<T>()) {
            return glm::identity<glm::tquat<T>>();
        }
        glm::tvec3<T> u = v / theta;
        return glm::tquat<T>(glm::cos(theta/2), glm::sin(theta/2) * u);
    }

    template <class T>
    inline T extractXRot(glm::tquat<T> q) {
        if (q.x * q.x + q.w * q.w <= glm::epsilon<T>()) {
            return 0;
        }
        return 2 * glm::atan(q.x, q.w);
    }

    template <class T>
    inline T extractYRot(glm::tquat<T> q) {
        if (q.y * q.y + q.w * q.w <= glm::epsilon<T>()) {
            return 0;
        }
        return 2 * glm::atan(q.y, q.w);
    }

    template <class T>
    inline T extractZRot(glm::tquat<T> q) {
        if (q.z * q.z + q.w * q.w <= glm::epsilon<T>()) {
            return 0;
        }
        return 2 * glm::atan(q.z, q.w);
    }

    template <class T>
    inline glm::tmat4x4<T> rotMatrixBetweenVecs(glm::tvec3<T> a, glm::tvec3<T> b) {
        glm::tvec3<T> v = glm::cross(a, b);
        T s2 = glm::dot(v, v);
        if (s2 < glm::epsilon<T>()) {
            return glm::tmat4x4<T>(1.0f);
        }
        else {
            // Rodrigue's formula
            T c = glm::dot(a, b);
            glm::tmat3x3<T> vhat;
            vhat[0][0] = vhat[1][1] = vhat[2][2] = 0;
            vhat[2][1] = v[0]; vhat[1][2] = -v[0];
            vhat[0][2] = v[1]; vhat[2][0] = -v[1];
            vhat[1][0] = v[2]; vhat[0][1] = -v[2];
            return glm::tmat3x3<T>(1) + vhat + vhat*vhat*(1 - c)/(s2);
        }
    }

    template <class T>
    // https://math.stackexchange.com/questions/90081/quaternion-distance
    inline glm::tquat<T> quatBetweenVecs(glm::tvec3<T> a, glm::tvec3<T> b) {
        glm::tvec3<T> w = glm::cross(a, b);
        glm::tquat<T> q = glm::tquat<T>(1.f + dot(a, b), w);
        return glm::normalize(q);
    }

    template <class T>
    inline T angleBetweenQuats(glm::tquat<T> q1, glm::tquat<T> q2) {
        T inner = glm::dot(q1, q2);
        T angle = glm::acos(2*inner*inner - 1);
        return inner;
    }

    template <class T>
    inline glm::tvec3<T> Ex() { return glm::tvec3<T>(1, 0, 0); }
    template <class T>
    inline glm::tvec3<T> Ey() { return glm::tvec3<T>(0, 1, 0); }
    template <class T>
    inline glm::tvec3<T> Ez() { return glm::tvec3<T>(0, 0, 1); }

    template <class T>
    glm::tmat3x3<T> Rx(T theta) {
        T s = glm::sin(theta); T c = glm::cos(theta);
        return glm::tmat3x3<T>(1, 0, 0, 0, c, s, 0, -s, c);
    }

    template <class T>
    glm::tmat3x3<T> Ry(T theta) {
        T s = glm::sin(theta); T c = glm::cos(theta);
        return glm::tmat3x3<T>(c, 0, -s, 0, 1, 0, s, 0, c);
    }

    template <class T>
    glm::tmat3x3<T> Rz(T theta) {
        T s = glm::sin(theta); T c = glm::cos(theta);
        return glm::tmat3x3<T>(c, s, 0, -s, c, 0, 0, 0, 1);
    }

    template <class T>
    glm::tquat<T> Rx_quat(T theta) {
        T s = glm::sin(theta/2); T c = glm::cos(theta/2);
        return glm::tquat<T>(c, s, 0, 0);
    }

    template <class T>
    glm::tquat<T> Ry_quat(T theta) {
        T s = glm::sin(theta/2); T c = glm::cos(theta/2);
        return glm::tquat<T>(c, 0, s, 0);
    }

    template <class T>
    glm::tquat<T> Rz_quat(T theta) {
        T s = glm::sin(theta/2); T c = glm::cos(theta/2);
        return glm::tquat<T>(c, 0, 0, s);
    }

    template <class T>
    inline glm::tmat3x3<T> mat3_from_diag(glm::tvec3<T> v) {
        return glm::tmat3x3<T>(v.x, 0, 0, 0, v.y, 0, 0, 0, v.z);
    }

    template <class T>
    inline glm::tmat3x3<T> cartesian_product(glm::tvec3<T> w, glm::tvec3<T> v) {
        glm::tmat3x3<T> m;
        m[0][0] = w[0]*v[0];
        m[0][1] = w[1]*v[0];
        m[0][2] = w[2]*v[0];
        m[1][0] = w[0]*v[1];
        m[1][1] = w[1]*v[1];
        m[1][2] = w[2]*v[1];
        m[2][0] = w[0]*v[2];
        m[2][1] = w[1]*v[2];
        m[2][2] = w[2]*v[2];
        return m;
    }

    template <class T>
    inline glm::tmat3x3<T> skew_symmetric(glm::tvec3<T> w) {
        glm::tmat3x3<T> m;
        m[0][0] = 0;
        m[1][1] = 0;
        m[2][2] = 0;
        m[1][2] = w.x;
        m[2][1] = -w.x;
        m[2][0] = w.y;
        m[0][2] = -w.y;
        m[0][1] = w.z;
        m[1][0] = -w.z;
        return m;
    }

    template <class T>
    inline glm::tvec3<T> skew_symmetric_cast(const glm::tmat3x3<T>& m) {
        return glm::tvec3<T>(m[1][2], m[2][0], m[0][1]);
    }

    template <class T>
    inline T tr(const glm::tmat3x3<T>& m) {
        return m[0][0] + m[1][1] + m[2][2];
    }

    // TODO: optimize this
    template <class T>
    inline glm::tmat3x3<T> exp_mat(glm::tvec3<T> v) {
        T theta = glm::length(v);
        glm::tmat3x3<T> R = glm::identity<glm::tmat3x3<T>>();
        if (theta <= glm::epsilon<T>()) {
            return R;
        }
        glm::tvec3<T> u = v / theta;
        glm::tmat3x3<T> K = skew_symmetric(u);
        R += glm::sin(theta) * K;
        R += (T(1) - glm::cos(theta)) * K * K;
        return R;
    }

    template <class T>
    inline glm::tvec3<T> log_mat(const glm::tmat3x3<T> &R) {
        auto v = glm::tvec3<T>(R[1][2] - R[2][1], R[2][0] - R[0][2], R[0][1] - R[1][0]);
        T cos_theta = (R[0][0] + R[1][1] + R[2][2] - 1) / 2;
        if (glm::epsilonEqual(cos_theta, T(1), T(1e-6))) {
            return T(0.5) * v;
        }
        else {
            T theta = glm::acos(cos_theta);
            return v * (theta / (2*glm::sin(theta)));
        }
    }

    template <class T>
    inline T length2(const glm::tmat3x3<T>& M) {
        return glm::length2(M[0]) + glm::length2(M[1]) + glm::length2(M[2]);
    }

    template <class T>
    inline T length(const glm::tmat3x3<T>& M) {
        return sqrt(length2(M));
    }
}


#endif //ARTSIM_COMMON_H
