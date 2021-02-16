// Taken from https://gist.github.com/alexsr/5065f0189a7af13b2f3bc43d22aff62f

// This is a GLSL implementation of
// "Computing the Singular Value Decomposition of 3 x 3 matrices with
// minimal branching and elementary floating point operations"
// by Aleka McAdams et.al.
// http://pages.cs.wisc.edu/~sifakis/papers/SVD_TR1690.pdf

// This should also work on the CPU using glm
// Then you probably should use glm::quat instead of glm::tquat<T>
// and glm::tmat3x3<T>_cast to convert to glm::tmat3x3<T>.

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/mat3x3.hpp>

#ifndef ARTSIM_SVD_H
#define ARTSIM_SVD_H

// GAMMA = 3 + sqrt(8)
// C_STAR = cos(pi/8)
// S_STAR = sin(pi/8)

#define GAMMA 5.8284271247
#define C_STAR 0.9238795325
#define S_STAR 0.3826834323
#define SVD_EPS 0.0000001

namespace glmx {

template<class T>
glm::tvec2<T> approx_givens_quat(T s_pp, T s_pq, T s_qq) {
    T c_h = 2 * (s_pp - s_qq);
    T s_h2 = s_pq * s_pq;
    T c_h2 = c_h * c_h;
    if (GAMMA * s_h2 < c_h2) {
        T omega = 1.0f / sqrt(s_h2 + c_h2);
        return glm::tvec2<T>(omega * c_h, omega * s_pq);
    }
    return glm::tvec2<T>(C_STAR, S_STAR);
}

template<class T>
glm::tmat3x3<T> symmetric_eigenanalysis(glm::tmat3x3<T> A) {
    glm::tmat3x3<T> S = transpose(A) * A;
    // jacobi iteration
    glm::tmat3x3<T> q = glm::tmat3x3<T>(1.0f);
    for (int i = 0; i < 5; i++) {
        glm::tvec2<T> ch_sh = approx_givens_quat(S[0].x, S[0].y, S[1].y);
        glm::tquat<T> ch_sh_quat = glm::tquat<T>(ch_sh.x, 0, 0, ch_sh.y);
        glm::tmat3x3<T> q_mat = glm::mat3_cast(ch_sh_quat);
        S = transpose(q_mat) * S * q_mat;
        q = q * q_mat;

        ch_sh = approx_givens_quat(S[0].x, S[0].z, S[2].z);
        ch_sh_quat = glm::tquat<T>(ch_sh.x, 0, -ch_sh.y, 0);
        q_mat = glm::mat3_cast(ch_sh_quat);
        S = transpose(q_mat) * S * q_mat;
        q = q * q_mat;

        ch_sh = approx_givens_quat(S[1].y, S[1].z, S[2].z);
        ch_sh_quat = glm::tquat<T>(ch_sh.x, ch_sh.y, 0, 0);
        q_mat = glm::mat3_cast(ch_sh_quat);
        S = transpose(q_mat) * S * q_mat;
        q = q * q_mat;

    }
    return q;
}

template<class T>
glm::tvec2<T> approx_qr_givens_quat(T a0, T a1) {
    T rho = sqrt(a0 * a0 + a1 * a1);
    T s_h = a1;
    T max_rho_eps = rho;
    if (rho <= SVD_EPS) {
        s_h = 0;
        max_rho_eps = SVD_EPS;
    }
    T c_h = max_rho_eps + a0;
    if (a0 < 0) {
        T temp = c_h - 2 * a0;
        c_h = s_h;
        s_h = temp;
    }
    T omega = T(1.0) / sqrt(c_h * c_h + s_h * s_h);
    return glm::tvec2<T>(omega * c_h, omega * s_h);
}

template<class T>
struct QR_mats {
    glm::tmat3x3<T> Q;
    glm::tmat3x3<T> R;
};

template<class T>
QR_mats<T> qr_decomp(glm::tmat3x3<T> B) {
    QR_mats<T> qr_decomp_result;
    glm::tmat3x3<T> R;
    // 1 0
    // (ch, 0, 0, sh)
    glm::tvec2<T> ch_sh10 = approx_qr_givens_quat(B[0].x, B[0].y);
    glm::tmat3x3<T> Q10 = glm::mat3_cast(glm::tquat<T>(ch_sh10.x, 0, 0, ch_sh10.y));
    R = transpose(Q10) * B;

    // 2 0
    // (ch, 0, -sh, 0)
    glm::tvec2<T> ch_sh20 = approx_qr_givens_quat(R[0].x, R[0].z);
    glm::tmat3x3<T> Q20 = glm::mat3_cast(glm::tquat<T>(ch_sh20.x, 0, -ch_sh20.y, 0));
    R = transpose(Q20) * R;

    // 2 1
    // (ch, sh, 0, 0)
    glm::tvec2<T> ch_sh21 = approx_qr_givens_quat(R[1].y, R[1].z);
    glm::tmat3x3<T> Q21 = glm::mat3_cast(glm::tquat<T>(ch_sh21.x, ch_sh21.y, 0, 0));
    R = transpose(Q21) * R;

    qr_decomp_result.R = R;

    qr_decomp_result.Q = Q10 * Q20 * Q21;
    return qr_decomp_result;
}

template<class T>
struct SVD_mats {
    glm::tmat3x3<T> U;
    glm::tmat3x3<T> Sigma;
    glm::tmat3x3<T> V;
};

template<class T>
SVD_mats<T> svd(glm::tmat3x3<T> A) {
    SVD_mats<T> svd_result;
    svd_result.V = symmetric_eigenanalysis(A);

    glm::tmat3x3<T> B = A * svd_result.V;

    // sort singular values
    T rho0 = glm::dot(B[0], B[0]);
    T rho1 = glm::dot(B[1], B[1]);
    T rho2 = glm::dot(B[2], B[2]);
    if (rho0 < rho1) {
        glm::tvec3<T> temp = B[1];
        B[1] = -B[0];
        B[0] = temp;
        temp = svd_result.V[1];
        svd_result.V[1] = -svd_result.V[0];
        svd_result.V[0] = temp;
        T temp_rho = rho0;
        rho0 = rho1;
        rho1 = temp_rho;
    }
    if (rho0 < rho2) {
        glm::tvec3<T> temp = B[2];
        B[2] = -B[0];
        B[0] = temp;
        temp = svd_result.V[2];
        svd_result.V[2] = -svd_result.V[0];
        svd_result.V[0] = temp;
        rho2 = rho0;
    }
    if (rho1 < rho2) {
        glm::tvec3<T> temp = B[2];
        B[2] = -B[1];
        B[1] = temp;
        temp = svd_result.V[2];
        svd_result.V[2] = -svd_result.V[1];
        svd_result.V[1] = temp;
    }

    QR_mats QR = qr_decomp(B);
    svd_result.U = QR.Q;
    svd_result.Sigma = QR.R;
    return svd_result;
}

template<class T>
struct UP_mats {
    glm::tmat3x3<T> U;
    glm::tmat3x3<T> P;
};

template<class T>
UP_mats<T> SVD_to_polar(SVD_mats<T> B) {
    UP_mats<T> polar;
    polar.P = B.V * B.Sigma * transpose(B.V);
    polar.U = B.U * transpose(B.V);
    return polar;
}

template<class T>
UP_mats<T> polar_decomp(glm::tmat3x3<T> A) {
    SVD_mats<T> B = svd(A);
    UP_mats<T> polar;
    polar.P = B.V * B.Sigma * transpose(B.V);
    polar.U = B.U * transpose(B.V);
    return polar;
}

}
#endif //ARTSIM_SVD_H
