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

        ttransform() : v(0), R(glm::identity<glm::tmat3x3<T>>()) {}
        explicit ttransform(glm::tvec3<T> v) : v(v), R(glm::identity<glm::tmat3x3<T>>()) {}
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
    inline ttransform<T> operator*(const glm::tmat3x3<T>& R, const ttransform<T>& t) {
        return {R * t.v, R * t.R};
    }

    template <class T>
    inline ttransform<T> operator*(const ttransform<T>& t1, const glm::tmat3x3<T>& R) {
        return {t1.v, t1.R * R};
    }

    template <class T>
    inline ttransform<T> operator*(glm::tvec3<T> v, const ttransform<T>& t) {
        return {t.v + v, t.R};
    }

    template <class T>
    inline ttransform<T> operator*(const ttransform<T>& t, glm::tvec3<T> v) {
        return {t.R * v + t.v, t.R};
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
        tscrew(T wx, T wy, T wz, T vx, T vy, T vz) : w(wx, wy, wz), v(vx, vy, vz) {}

        template <class U>
        explicit operator tscrew<U>() const { return tscrew<U>(w, v); }

        // Use this with care!
        const T& operator[](size_t i) const { return reinterpret_cast<const T*>(this)[i]; }
        T& operator[](size_t i) { return reinterpret_cast<T*>(this)[i]; }
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

    // 3x3 symmetric matrix.
    template <class T>
    struct tsmat3x3 {
        T xx, yy, zz, yz, zx, xy;

        tsmat3x3() = default;
        tsmat3x3(T k) : xx(k), yy(k), zz(k), yz(0), zx(0), xy(0) {}
        tsmat3x3(T xx, T yy, T zz, T yz, T zx, T xy) : xx(xx), yy(yy), zz(zz), yz(yz), zx(zx), xy(xy) {}

        template <class U>
        explicit operator tsmat3x3<U>() const { return tsmat3x3<U>(xx, yy, zz, yz, zx, xy); }


        glm::tvec3<T> operator[](size_t i) const {
            assert(i >= 0 && i < 3);
            switch(i) {
                case 0: return glm::tvec3<T>(xx, xy, zx);
                case 1: return glm::tvec3<T>(xy, yy, yz);
                case 2: return glm::tvec3<T>(zx, yz, zz);
                default: return glm::tvec3<T>(0);
            }
        }
    };

    template <class T>
    inline glm::tmat3x3<T> mat3_cast(const tsmat3x3<T>& m) {
        return {m.xx, m.xy, m.zx, m.xy, m.yy, m.yz, m.zx, m.yz, m.zz};
    }

    template <class T>
    inline tsmat3x3<T> smat3_cast(const glm::tmat3x3<T>& m) {
        return {m[0][0], m[1][1], m[2][2], m[1][2], m[2][0], m[0][1]};
    }


    template <class T>
    inline tsmat3x3<T> operator-(const tsmat3x3<T>& m) {
        return {-m.xx, -m.yy, -m.zz, -m.yz, -m.zx, -m.xy};
    }

    template <class T>
    inline tsmat3x3<T> operator+(const tsmat3x3<T>& m1, const tsmat3x3<T>& m2) {
        return {m1.xx + m2.xx, m1.yy + m2.yy, m1.zz + m2.zz, m1.yz + m2.yz, m1.zx + m2.zx, m1.xy + m2.xy};
    }

    template <class T>
    inline tsmat3x3<T> operator-(const tsmat3x3<T>& m1, const tsmat3x3<T>& m2) {
        return {m1.xx - m2.xx, m1.yy - m2.yy, m1.zz - m2.zz, m1.yz - m2.yz, m1.zx - m2.zx, m1.xy - m2.xy};
    }

    template <class T>
    inline tsmat3x3<T>& operator+=(tsmat3x3<T>& m1, const tsmat3x3<T>& m2) {
        m1.xx += m2.xx; m1.yy += m2.yy; m1.zz += m2.zz; m1.yz += m2.yz; m1.zx += m2.zx; m1.xy += m2.xy;
        return m1;
    }

    template <class T>
    inline tsmat3x3<T>& operator-=(tsmat3x3<T>& m1, const tsmat3x3<T>& m2) {
        m1.xx -= m2.xx; m1.yy -= m2.yy; m1.zz -= m2.zz; m1.yz -= m2.yz; m1.zx -= m2.zx; m1.xy -= m2.xy;
        return m1;
    }

    template <class T>
    inline glm::tvec3<T> operator*(const tsmat3x3<T>& m, const glm::tvec3<T>& v) {
        return {m.xx*v.x + m.xy*v.y + m.zx*v.z, m.xy*v.x + m.yy*v.y + m.yz*v.z, m.zx*v.x + m.yz*v.y + m.zz*v.z};
    }

    template <class T>
    inline tsmat3x3<T> operator*(T k, const tsmat3x3<T>& m) {
        return {m.xx * k, m.yy * k, m.zz * k, m.yz * k, m.zx * k, m.xy * k};
    }

    template <class T>
    inline tsmat3x3<T> operator/(const tsmat3x3<T>& m, T k) {
        return {m.xx / k, m.yy / k, m.zz / k, m.yz / k, m.zx / k, m.xy / k};
    }

    template <class T>
    inline tsmat3x3<T>& operator*=(tsmat3x3<T>& m, T k) {
        m.xx *= k; m.yy *= k; m.zz *= k; m.yz *= k; m.zx *= k; m.xy *= k;
        return m;
    }

    template <class T>
    inline tsmat3x3<T>& operator/=(tsmat3x3<T>& m, T k) {
        m.xx /= k; m.yy /= k; m.zz /= k; m.yz /= k; m.zx /= k; m.xy /= k;
        return m;
    }

    template <class T>
    tsmat3x3<T> inverse(const tsmat3x3<T>& m) {
        tsmat3x3<T> minv;
        minv.xx = m.yy * m.zz - m.yz * m.yz;
        minv.yy = m.zz * m.xx - m.zx * m.zx;
        minv.zz = m.xx * m.yy - m.xy * m.xy;
        minv.xy = m.zx * m.yz - m.zz * m.xy;
        minv.yz = m.xy * m.zx - m.xx * m.yz;
        minv.zx = m.yz * m.xy - m.yy * m.zx;
        T determinant = m.xx * minv.xx + m.xy * minv.xy + m.zx * minv.zx;
        minv /= determinant;
        return minv;
    }

    // TODO: optimize rotate and inv_rotate (see Featherstone2008 A.5)
    template <class T>
    inline tsmat3x3<T> rotate(const tsmat3x3<T>& I, const glm::tmat3x3<T>& R) {
        return smat3_cast(R * mat3_cast(I) * glm::transpose(R));
    }

    template <class T>
    inline tsmat3x3<T> rotate_x(const tsmat3x3<T>& I, const glm::tmat3x3<T>& R) {
        T c = R[1][1]; T s = R[1][2];
        T cs = s * c;
        T ss = s * s;
        T alpha = 2*cs*I.yz + ss*(I.zz - I.yy);
        T beta = cs*(I.zz - I.yy) + (1 - 2*ss) * I.yz;
        return tsmat3x3(I.xx, I.yy + alpha, I.zz - alpha, beta, c*I.zx - s*I.xy, c*I.xy + s*I.zx);
    }

    template <class T>
    inline tsmat3x3<T> rotate_y(const tsmat3x3<T>& I, const glm::tmat3x3<T>& R) {
        T c = R[2][2]; T s = R[2][0];
        T cs = s * c;
        T ss = s * s;
        T alpha = 2*cs*I.zx + ss*(I.xx - I.zz);
        T beta = cs*(I.xx - I.zz) + (1 - 2*ss) * I.zx;
        return tsmat3x3(I.xx - alpha, I.yy, I.zz + alpha, c*I.yz + s*I.xy, beta, c*I.xy - s*I.yz);
    }

    template <class T>
    inline tsmat3x3<T> rotate_z(const tsmat3x3<T>& I, const glm::tmat3x3<T>& R) {
        T c = R[0][0]; T s = R[0][1];
        T cs = s * c;
        T ss = s * s;
        T alpha = 2*cs*I.xy + ss*(I.yy - I.xx);
        T beta = cs*(I.yy - I.xx) + (1 - 2*ss) * I.xy;
        return tsmat3x3(I.xx + alpha, I.yy - alpha, I.zz, c*I.yz - s*I.zx, c*I.zx + s*I.yz, beta);
    }

    template <class T>
    inline tsmat3x3<T> inv_rotate(const tsmat3x3<T>& I, const glm::tmat3x3<T>& R) {
        return smat3_cast(glm::transpose(R) * mat3_cast(I) * R);
    }

    template <class T>
    inline glm::tmat3x3<T> rotate(const glm::tmat3x3<T>& M, const glm::tmat3x3<T>& R) {
        return R * M * glm::transpose(R);
    }

    template <class T>
    inline glm::tmat3x3<T> inv_rotate(const glm::tmat3x3<T>& M, const glm::tmat3x3<T>& R) {
        return glm::transpose(R) * M * R;
    }

    template <class T>
    inline tsmat3x3<T> symmetric_cartesian_product(glm::tvec3<T> w) {
        return {w[0]*w[0], w[1]*w[1], w[2]*w[2], w[1]*w[2], w[2]*w[0], w[0]*w[1]};
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
        tsmat3x3<T> I;
        glm::tvec3<T> c;
        T m;

        tspmat() = default;
        tspmat(tsmat3x3<T> I, glm::tvec3<T> c, T m) : I(I), c(c), m(m) {}

        template <class U>
        explicit operator tspmat<U>() const { return tspmat<U>(I, c, m); }
    };

    template <class T>
    inline tscrew<T> operator*(const tspmat<T>& G, const tscrew<T>& V) {
        return tscrew<T>(G.I * V.w + G.m * glm::cross(G.c, V.v), G.m*(V.v - glm::cross(G.c, V.w)));
    }

    template <class T>
    inline tspmat<T> inv_transform(const tspmat<T>& G_b, const ttransform<T>& T_ba) {
        tspmat<T> G_a;
        glm::tvec3<T> c = glm::transpose(T_ba.R) * G_b.c;
        glm::tvec3<T> cp = glm::transpose(T_ba.R) * (G_b.c - T_ba.v);
        G_a.I = inv_rotate(G_b.I, T_ba.R);
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
    inline T quadratic_form(const tspmat<T>& G, const tscrew<T>& V) {
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
        tsmat3x3<T> I, M;
        glm::tmat3x3<T> C;

        tsmat6x6() = default;
        tsmat6x6(tsmat3x3<T> I, glm::tmat3x3<T> C, tsmat3x3<T> M) : I(I), C(C), M(M) {}
        explicit tsmat6x6(const tspmat<T>& G)
                : I(G.I), C(G.m * skew_symmetric(G.c)), M(G.m, G.m, G.m, 0, 0, 0) {}

        template <class U>
        explicit operator tsmat6x6<U>() const { return tsmat6x6<U>(I, C, M); }

        // Use with care!
        tscrew<T> operator[](size_t i) const {
            assert(i >= 0 && i < 6);
            if (i < 3) {
                return tscrew<T>(I[i], glm::tvec3<T>(C[0][i], C[1][i], C[2][i]));
            }
            else {
                return tscrew<T>(C[i-3], M[i-3]);
            }
        }
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
    inline tsmat6x6<T> inv_transform(const tsmat6x6<T>& G_b, const ttransform<T>& T_ba) {
        tsmat6x6<T> G_a;
        glm::tmat3x3<T> P = skew_symmetric(T_ba.v);
        glm::tmat3x3<T> PM = P * mat3_cast(G_b.M);
        glm::tmat3x3<T> CP = G_b.C * P;
        G_a.I = inv_rotate(G_b.I + smat3_cast(CP + glm::transpose(CP) - PM*P), T_ba.R);
        G_a.C = inv_rotate(G_b.C - PM, T_ba.R);
        G_a.M = inv_rotate(G_b.M, T_ba.R);
        return G_a;
    }

    template <class T>
    inline tsmat6x6<T> inv_transform(const tsmat3x3<T>& M_b, const ttransform<T>& T_ba) {
        tsmat6x6<T> G_a;
        glm::tmat3x3<T> P = skew_symmetric(T_ba.v);
        glm::tmat3x3<T> PM = P * mat3_cast(M_b);
        G_a.I = -inv_rotate(smat3_cast(PM*P), T_ba.R);
        G_a.C = -inv_rotate(PM, T_ba.R);
        G_a.M = inv_rotate(M_b, T_ba.R);
        return G_a;
    }

    template <class T>
    inline T quadratic_form(const tsmat6x6<T>& G, tscrew<T> V) {
        return quadratic_form(G.I, V.w) + quadratic_form(G.M, V.v) + 2*quadratic_form(G.C, V.w, V.v);
    }

    template <class T>
    inline tsmat6x6<T> inverse(const tsmat6x6<T>& G) {
        tsmat3x3<T> Iinv = inverse(G.I);
        glm::tmat3x3<T> Iinv_C = mat3_cast(Iinv) * G.C;
        tsmat3x3<T> D = inverse(G.M - smat3_cast(glm::transpose(G.C) * Iinv_C));

        tsmat6x6<T> Ginv;
        Ginv.I = Iinv + smat3_cast(Iinv_C * mat3_cast(D) * glm::transpose(Iinv_C));
        Ginv.C = -Iinv_C * mat3_cast(D);
        Ginv.M = D;
        return Ginv;
    }

    using transform = ttransform<float>;
    using screw = tscrew<float>;
    using spmat = tspmat<float>;
    using smat6x6 = tsmat6x6<float>;
    using smat3x3 = tsmat3x3<float>;
    using mat6x6 = glm::mat<6, 6, float>;
}


#endif //ARTSIM_SE3_H
