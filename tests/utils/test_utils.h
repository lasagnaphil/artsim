//
// Created by Phillip Chang on 2020/09/26.
//

#ifndef ARTSIM_TEST_UTILS_H
#define ARTSIM_TEST_UTILS_H

#include <artsim/math/se3.h>

#include <Eigen/Dense>
#include <glm/matrix.hpp>
#include <random>

template <class T>
void populate_random(std::default_random_engine& engine, T* buf, size_t size) {
    for (int i = 0; i < size; i++) {
        buf[i] = std::uniform_real_distribution<T>(-1, 1)(engine);
    }
}

template <class T, class Real>
void get_random(std::default_random_engine& engine, T& obj) {
    size_t size = sizeof(T) / sizeof(Real);
    Real* ptr = reinterpret_cast<Real*>(&obj);
    for (int i = 0; i < size; i++) {
        ptr[i] = std::uniform_real_distribution<Real>(-1, 1)(engine);
    }
}

template <class Real>
void get_random_symmetric(std::default_random_engine& engine, glm::tmat3x3<Real>& M) {
    M[0][0] = std::uniform_real_distribution<Real>(-1, 1)(engine);
    M[0][1] = std::uniform_real_distribution<Real>(-1, 1)(engine);
    M[0][2] = std::uniform_real_distribution<Real>(-1, 1)(engine);
    M[1][0] = M[0][1];
    M[1][1] = std::uniform_real_distribution<Real>(-1, 1)(engine);
    M[1][2] = std::uniform_real_distribution<Real>(-1, 1)(engine);
    M[2][0] = M[0][2];
    M[2][1] = M[1][2];
    M[2][2] = std::uniform_real_distribution<Real>(-1, 1)(engine);
}

template <class Real>
void get_random(std::default_random_engine& engine, artsim::tsmat6x6<Real>& M) {
    get_random_symmetric<Real>(engine, M.I);
    get_random<glm::tmat3x3<Real>, Real>(engine, M.C);
    get_random_symmetric<Real>(engine, M.M);
}

template <class Real>
void get_random(std::default_random_engine& engine, glm::tquat<Real>& q) {
    q[0] = std::uniform_real_distribution<Real>(-1, 1)(engine);
    q[1] = std::uniform_real_distribution<Real>(-1, 1)(engine);
    q[2] = std::uniform_real_distribution<Real>(-1, 1)(engine);
    q[3] = std::uniform_real_distribution<Real>(-1, 1)(engine);
}

template <class Real>
void get_random(std::default_random_engine& engine, artsim::ttransform<Real>& T) {
    get_random<glm::tvec3<Real>, Real>(engine, T.v);
    get_random<glm::tquat<Real>, Real>(engine, T.q);
}

template <class T, class U>
void populate_random(std::default_random_engine& engine, T* obj) {
    size_t size = sizeof(T) / sizeof(U);
    U* ptr = reinterpret_cast<U*>(obj);
    for (int i = 0; i < size; i++) {
        ptr[i] = std::uniform_real_distribution<U>(-1, 1)(engine);
    }
}

template <class T, class U>
void populate_random(std::default_random_engine& engine, artsim::ttransform<T>* obj) {
    size_t size = sizeof(T) / sizeof(U);
    U* ptr = reinterpret_cast<U*>(obj);
    for (int i = 0; i < size; i++) {
        ptr[i] = std::uniform_real_distribution<U>(-1, 1)(engine);
    }
    obj->q = glm::normalize(obj->q);
}


template <class T, int R, int C>
Eigen::Matrix<T, R, C> to_eigen(const glm::mat<C, R, T>& M) {
    Eigen::Matrix<T, R, C> Me;
    for (int i = 0; i < R; i++) {
        for (int j = 0; j < C; j++) {
            Me(i, j) = M[j][i];
        }
    }
    return Me;
}

#define compare_eigen(M1, M2) \
for (int i = 0; i < M1.rows(); i++) { \
    for (int j = 0; j < M1.cols(); j++) { \
        INFO("Index: " << i << ", " << j); \
        CHECK(M1(i, j) == doctest::Approx(M2(i, j))); \
    } \
}


template <class T>
Eigen::Matrix<T, 6, 1> to_eigen(const artsim::tscrew<T>& S) {
    return Eigen::Map<Eigen::Matrix<T, 6, 1>>((T*)&S);
}

template <class T>
Eigen::Matrix<T, 6, 3> to_eigen(artsim::tscrew<T> S[3]) {
    Eigen::Matrix<T, 6, 3> M;
    M.col(0) = to_eigen(S[0]);
    M.col(1) = to_eigen(S[1]);
    M.col(2) = to_eigen(S[2]);
    return M;
}

template <class T>
Eigen::Matrix<T, 6, 6> to_eigen(const artsim::tsmat6x6<T>& A) {
    Eigen::Matrix<T, 6, 6> Ae;
    Ae.template block<3, 3>(0, 0) = to_eigen(A.I);
    Ae.template block<3, 3>(0, 3) = to_eigen(A.C);
    Ae.template block<3, 3>(3, 0) = to_eigen(A.C).transpose();
    Ae.template block<3, 3>(3, 3) = to_eigen(A.M);
    return Ae;
}

template <class T>
Eigen::Matrix<T, 6, 6> to_eigen_adj_matrix(const artsim::ttransform<T>& t) {
    Eigen::Matrix<T, 6, 6> M;
    auto R = glm::mat3_cast(t.q);
    auto P = artsim::skew_symmetric(t.v);
    M.template block<3, 3>(0, 0) = to_eigen(R);
    M.template block<3, 3>(0, 3) = Eigen::Matrix<T, 3, 3>::Zero();
    M.template block<3, 3>(3, 0) = to_eigen(P*R);
    M.template block<3, 3>(3, 3) = to_eigen(R);
    return M;
}

#endif //ARTSIM_TEST_UTILS_H
