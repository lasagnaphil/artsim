//
// Created by lasagnaphil on 20. 9. 10..
//

#ifndef ARTSIM_SE3_H
#define ARTSIM_SE3_H

#include "common.h"

#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/norm.hpp>

namespace artsim {
    template <class T>
    struct ttransform {
        glm::tvec3<T> v;
        glm::tmat3x3<T> R;

        ttransform() : v(0), R(1, 0, 0, 0, 1, 0, 0, 0, 1) {}
        explicit ttransform(glm::tvec3<T> v) : v(v), R(1, 0, 0, 0, 1, 0, 0, 0, 1) {}
        explicit ttransform(glm::tquat<T> q) : v(0), R(glm::mat3_cast(q)) {}
        explicit ttransform(glm::tmat3x3<T> R) : v(0), R(R) {}
        ttransform(glm::tvec3<T> v, glm::tmat3x3<T> R) : v(v), R(R) {}

        template <class U>
        explicit operator ttransform<U>() const { return ttransform<U>(v, R); }
    };

    template <class T>
    inline typename glm::tmat4x4<T> mat4_cast(const ttransform<T> &t) {
        glm::tmat4x4<T> m;
        m[0] = glm::tvec4<T>(t.R[0], 0);
        m[1] = glm::tvec4<T>(t.R[1], 0);
        m[2] = glm::tvec4<T>(t.R[2], 0);
        m[3] = glm::tvec4<T>(t.v, 1);
        return m;
    }

    template <class T>
    inline ttransform<T> operator*(const ttransform<T> &t1, const ttransform<T> &t2) {
        return {t1.R * t2.v + t1.v, t1.R * t2.R};
    }

    template <class T>
    inline ttransform<T> operator*(glm::quat q, const ttransform<T>& t) {
        return {glm::mat3_cast(q) * t.R, glm::mat3_cast(q) * t.R};
    }

    template <class T>
    inline ttransform<T> operator*(glm::tmat3x3<T> R, const ttransform<T>& t) {
        return {R * t.R, R * t.R};
    }

    template <class T>
    inline ttransform<T> operator*(glm::tvec3<T> v, const ttransform<T>& t) {
        return {t.v + v, t.R};
    }

    template <class T>
    inline ttransform<T> operator/(const ttransform<T> &t1, const ttransform<T> &t2) {
        return {glm::transpose(t2.R) * (t1.v - t2.v), glm::transpose(t2.R) * t1.R};
    }

    template <class T>
    inline ttransform<T> inverse(const ttransform<T>& t) {
        return ttransform<T>(glm::transpose(t.R) * (-t.v), glm::transpose(t.R));
    }

    template <class T>
    struct tscrew {
        glm::tvec3<T> w, v;

        tscrew() : w(0), v(0) {}
        tscrew(glm::tvec3<T> w, glm::tvec3<T> v) : w(w), v(v) {}

        template <class U>
        explicit operator tscrew<U>() const { return tscrew<U>(w, v); }
    };

    template <class T>
    inline tscrew<T> make_tscrew(const T* ptr) {
        return tscrew<T>(glm::tvec3<T>(ptr[0], ptr[1], ptr[2]), glm::tvec3<T>(ptr[3], ptr[4], ptr[5]));
    }

    template <class T>
    inline tscrew<T> operator+(const tscrew<T>& V1, const tscrew<T>& V2) {
        return {V1.w + V2.w, V1.v + V2.v};
    }

    template <class T>
    inline tscrew<T>& operator+=(tscrew<T>& V1, const tscrew<T>& V2) {
        V1.w += V2.w; V1.v += V2.v;
        return V1;
    }

    template <class T>
    inline tscrew<T> operator-(const tscrew<T>& V1, const tscrew<T>& V2) {
        return {V1.w - V2.w, V1.v - V2.v};
    }

    template <class T>
    inline tscrew<T>& operator-=(tscrew<T>& V1, const tscrew<T>& V2) {
        V1.w -= V2.w; V1.v -= V2.v;
        return V1;
    }

    template <class T>
    inline tscrew<T> operator*(const tscrew<T>& V, T theta) {
        return {theta * V.w, theta * V.v};
    }

    template <class T>
    inline tscrew<T> operator*(T theta, const tscrew<T>& V) {
        return {theta * V.w, theta * V.v};
    }

    template <class T>
    inline tscrew<T> operator/(const tscrew<T>& V, T theta) {
        return {V.w / theta, V.v / theta};
    }

    template <class T>
    inline tscrew<T> operator-(const tscrew<T>& V) {
        return {-V.w, -V.v};
    }

    template <class T>
    inline T dot(const tscrew<T>& V1, const tscrew<T>& V2) {
        return glm::dot(V1.w, V2.w) + glm::dot(V1.v, V2.v);
    }

    template <class T>
    inline T length(const tscrew<T>& V) {
        T w2 = glm::length2(V.w);
        if (w2 >= glm::epsilon<T>()) {
            return glm::sqrt(w2);
        }
        T v2 = glm::length2(V.v);
        if (v2 >= glm::epsilon<T>()) {
            return glm::sqrt(v2);
        }
        return 0;
    }

    template <class T>
    inline tscrew<T> normalize(const tscrew<T>& V) {
        T w2 = glm::length2(V.w);
        if (w2 >= glm::epsilon<T>()) {
            return V / glm::sqrt(w2);
        }
        T v2 = glm::length2(V.v);
        if (v2 >= glm::epsilon<T>()) {
            return V / glm::sqrt(v2);
        }
        return tscrew<T>();
    }

    // Calculates exp([V] * theta).
    template <class T>
    inline ttransform<T> move(tscrew<T> V, T theta) {
        glm::tvec3<T> w_cross_v = glm::cross(V.w, V.v);
        glm::tvec3<T> p = V.v * theta + (1 - glm::cos(theta)) * w_cross_v +
                      (theta - glm::sin(theta)) * glm::cross(V.w, w_cross_v);
        glm::tmat3x3<T> R = artsim::exp_mat(V.w * theta);

        return ttransform<T>(p, R);
    }

    template <class T>
    inline tscrew<T> Ad(ttransform<T> t, tscrew<T> V) {
        glm::tvec3<T> w = t.R * V.w;
        return tscrew<T>(w, glm::cross(t.v, w) + t.R * V.v);
    }

    template <class T>
    inline tscrew<T> AdT(ttransform<T> t, tscrew<T> V) {
        return tscrew<T>(glm::transpose(t.R) * (V.w + glm::cross(V.v, t.v)), glm::transpose(t.R) * V.v);
    }

    template <class T>
    inline tscrew<T> ad(tscrew<T> V1, tscrew<T> V2) {
        return tscrew<T>(glm::cross(V1.w, V2.w), glm::cross(V1.v, V2.w) + glm::cross(V1.w, V2.v));
    }

    template <class T>
    inline tscrew<T> adT(tscrew<T> V1, tscrew<T> V2) {
        return tscrew<T>(glm::cross(V2.w, V1.w) + glm::cross(V2.v, V1.v), glm::cross(V2.v, V1.w));
    }

    template <class T>
    inline glm::tmat3x3<T> move_frame(glm::tmat3x3<T> I_b, glm::tquat<T> q_ba) {
        glm::tmat3x3<T> R = glm::mat3_cast(q_ba);
        return glm::transpose(R) * I_b * R;
    }

    template <class T>
    inline glm::tmat3x3<T> move_frame(glm::tmat3x3<T> I_b, glm::tmat3x3<T> R_ba) {
        return glm::transpose(R_ba) * I_b * R_ba;
    }

    template <class T>
    inline T quadratic_form(const glm::tmat3x3<T>& I, glm::tvec3<T> w, glm::tvec3<T> v) {
        return I[0][0]*w[0]*v[0] + I[0][1]*w[1]*v[0] + I[0][2]*w[2]*v[0]
               + I[1][0]*w[0]*v[1] + I[1][1]*w[1]*v[1] + I[1][2]*w[2]*v[1]
               + I[2][0]*w[0]*v[2] + I[2][1]*w[1]*v[2] + I[2][2]*w[2]*v[2];
    }

    template <class T>
    inline T quadratic_form(const glm::tmat3x3<T>& I, glm::tvec3<T> w) {
        return I[0][0]*w[0]*w[0] + I[0][1]*w[1]*w[0] + I[0][2]*w[2]*w[0]
               + I[1][0]*w[0]*w[1] + I[1][1]*w[1]*w[1] + I[1][2]*w[2]*w[1]
               + I[2][0]*w[0]*w[2] + I[2][1]*w[1]*w[2] + I[2][2]*w[2]*w[2];
    }

    template <class T>
    inline glm::tmat3x3<T> symmetric_cartesian_product(glm::tvec3<T> w) {
        return cartesian_product(w, w);
    }

    // Spatial matrix.
    /*
     *      Spatial mass matrix
           ---------- ----------
        0 |          |          |
        1 |  I(=mG)  |   m c×   |
        2 |          |          |
           ---------- ----------
        3 |          |          |
        4 |  -m c×   |    mE    |
        5 |          |          |
           ---------- ----------
     */
    template <class T>
    struct tspmat {
        glm::tmat3x3<T> I;
        glm::tvec3<T> c;
        T m;

        tspmat() = default;
        tspmat(glm::tmat3x3<T> I, glm::tvec3<T> c, T m) : I(I), c(c), m(m) {}

        template <class U>
        explicit operator tspmat<U>() const { return tspmat<U>(I, c, m); }
    };

    template <class T>
    inline tscrew<T> operator*(const tspmat<T>& G, tscrew<T> V) {
        return tscrew<T>(G.I * V.w + G.m * glm::cross(G.c, V.v), G.m*(V.v - glm::cross(G.c, V.w)));
    }

    template <class T>
    inline tspmat<T> move_frame(const tspmat<T>& G_b, ttransform<T> T_ba) {
        tspmat<T> G_a;
        glm::tvec3<T> c = glm::transpose(T_ba.R) * G_b.c;
        glm::tvec3<T> cp = glm::transpose(T_ba.R) * (G_b.c - T_ba.v);
        G_a.I = move_frame(G_b.I, T_ba.R);
        G_a.I.xx += G_b.m*(-c.y*c.y - c.z*c.z + cp.y*cp.y + cp.z*cp.z);
        G_a.I.yy += G_b.m*(-c.z*c.z - c.x*c.x + cp.z*cp.z + cp.x*cp.x);
        G_a.I.zz += G_b.m*(-c.x*c.x - c.y*c.y + cp.x*cp.x + cp.y*cp.y);
        G_a.I.xy += G_b.m*(c.x*c.y - cp.x*cp.y);
        G_a.I.yz += G_b.m*(c.y*c.z - cp.y*cp.z);
        G_a.I.zx += G_b.m*(c.z*c.x - cp.z*cp.x);
        G_a.c = cp;
        G_a.m = G_b.m;
        return G_a;
    }

    template <class T>
    inline T quadratic_form(const tspmat<T>& G, tscrew<T> V) {
        return quadratic_form(G.I, V.w) + G.m*glm::length2(V.v) - 2*G.m*glm::dot(glm::cross(V.w, V.v), G.c);
    }

    // Symmetric 6x6 matrix. (Articulation matrix)
    /*
     *     Symmetric 6x6 matrix
           ---------- ----------
        0 |          |          |
        1 |    I     |    C     |
        2 |          |          |
           ---------- ----------
        3 |          |          |
        4 |   C^T    |    M     |
        5 |          |          |
           ---------- ----------
     */
    template <class T>
    struct tsmat6x6 {
        glm::tmat3x3<T> I, M;
        glm::tmat3x3<T> C;

        tsmat6x6() = default;
        tsmat6x6(glm::tmat3x3<T> I, glm::tmat3x3<T> C, glm::tmat3x3<T> M) : I(I), C(C), M(M) {}
        explicit tsmat6x6(const tspmat<T>& G)
                : I(G.I), C(G.m * skew_symmetric(G.c)), M(G.m, 0, 0, 0, G.m, 0, 0, 0, G.m) {}

        template <class U>
        explicit operator tsmat6x6<U>() const { return tsmat6x6<U>(I, C, M); }
    };

    template <class T>
    inline tsmat6x6<T> operator+(const tsmat6x6<T>& G1, const tsmat6x6<T>& G2) {
        return tsmat6x6<T>(G1.I + G2.I, G1.C + G2.C, G1.M + G2.M);
    }

    template <class T>
    inline tsmat6x6<T>& operator+=(tsmat6x6<T>& G1, const tsmat6x6<T>& G2) {
        G1.I += G2.I; G1.C += G2.C; G1.M += G2.M;
        return G1;
    }

    template <class T>
    inline tsmat6x6<T> operator-(const tsmat6x6<T>& G1, const tsmat6x6<T>& G2) {
        return tsmat6x6<T>(G1.I - G2.I, G1.C - G2.C, G1.M - G2.M);
    }

    template <class T>
    inline tsmat6x6<T>& operator-=(tsmat6x6<T>& G1, const tsmat6x6<T>& G2) {
        G1.I -= G2.I; G1.C -= G2.C; G1.M -= G2.M;
        return G1;
    }

    template <class T>
    inline tsmat6x6<T> operator*(const tsmat6x6<T>& G, T k) {
        return tsmat6x6<T>(G.I*k, G.C*k, G.M*k);
    }

    template <class T>
    inline tsmat6x6<T> operator*(T k, const tsmat6x6<T>& G) {
        return tsmat6x6<T>(G.I*k, G.C*k, G.M*k);
    }

    template <class T>
    inline tsmat6x6<T> operator*=(tsmat6x6<T>& G, T k) {
        G.I *= k; G.C *= k; G.M *= k; return G;
    }

    template <class T>
    inline tsmat6x6<T> operator/(const tsmat6x6<T>& G, T k) {
        return tsmat6x6<T>(G.I/k, G.C/k, G.M/k);
    }

    template <class T>
    inline tsmat6x6<T> operator/=(tsmat6x6<T>& G, T k) {
        G.I /= k; G.C /= k; G.M /= k; return G;
    }

    template <class T>
    inline tscrew<T> operator*(const tsmat6x6<T>& G, tscrew<T> V) {
        return tscrew<T>(G.I * V.w + G.C * V.v, glm::transpose(G.C) * V.w + G.M * V.v);
    }

    template <class T>
    inline tsmat6x6<T> symmetric_cartesian_product(tscrew<T> V) {
        return tsmat6x6<T>(symmetric_cartesian_product(V.w),
                        cartesian_product(V.w, V.v),
                        symmetric_cartesian_product(V.v));
    }

    template <class T>
    inline tsmat6x6<T> move_frame(const tsmat6x6<T>& G_b, const ttransform<T>& T_ba) {
        tsmat6x6<T> G_a;
        glm::tmat3x3<T> P = skew_symmetric(T_ba.v);
        glm::tmat3x3<T> PM = P * G_b.M;
        glm::tmat3x3<T> CP = G_b.C * P;
        G_a.I = move_frame(G_b.I + CP + glm::transpose(CP) - PM*P, T_ba.R);
        G_a.C = move_frame(G_b.C - PM, T_ba.R);
        G_a.M = move_frame(G_b.M, T_ba.R);
        return G_a;
    }

    template <class T>
    inline T quadratic_form(const tsmat6x6<T>& G, tscrew<T> V) {
        return quadratic_form(G.I, V.w) + quadratic_form(G.M, V.v) + 2*quadratic_form(G.C, V.w, V.v);
    }

    template <class T>
    inline tsmat6x6<T> inverse(const tsmat6x6<T>& G) {
        glm::tmat3x3<T> Iinv = inverse(G.I);
        glm::tmat3x3<T> Iinv_C = Iinv * G.C;
        glm::tmat3x3<T> D = inverse(G.M - glm::transpose(G.C) * Iinv_C);

        tsmat6x6<T> Ginv;
        Ginv.I = Iinv + Iinv_C * D * glm::transpose(Iinv_C);
        Ginv.C = -Iinv_C * D;
        Ginv.M = D;
        return Ginv;
    }

    using transform = ttransform<float>;
    using screw = tscrew<float>;
    using spmat = tspmat<float>;
    using smat6x6 = tsmat6x6<float>;
    using mat6x6 = glm::mat<6, 6, float>;
}


#endif //ARTSIM_SE3_H
