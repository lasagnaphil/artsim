//
// Created by lasagnaphil on 7/23/21.
//

#ifndef EOS_SCAN_TO_HUMAN_EIGEN_H
#define EOS_SCAN_TO_HUMAN_EIGEN_H

#include <artsim/types.h>
#include <artsim/math/se3.h>
#include <artsim/math/dynmat.h>
#include <glm/gtc/type_ptr.hpp>
#include <Eigen/Dense>

namespace Eigen {

using MatrixXr = Eigen::Matrix<artsim::real, Eigen::Dynamic, Eigen::Dynamic>;
using VectorXr = Eigen::Matrix<artsim::real, Eigen::Dynamic, 1>;

using Matrix3r = Eigen::Matrix<artsim::real, 3, 3>;
using Vector3r = Eigen::Matrix<artsim::real, 3, 1>;

}

using Eigen::MatrixXr;
using Eigen::VectorXr;
using Eigen::Matrix3r;
using Eigen::Vector3r;

template <class T, int Rows, int Cols>
inline Eigen::Matrix<T, Rows, Cols> glm_to_eigen(const glm::mat<Cols, Rows, T>& M) {
    Eigen::Matrix<T, Rows, Cols> ret;
    memcpy(ret.data(), glm::value_ptr(M), sizeof(T) * Rows * Cols);
    return ret;
}

template <class T, int Rows>
inline Eigen::Matrix<T, Rows, 1> glm_to_eigen(const glm::vec<Rows, T>& v) {
    Eigen::Matrix<T, Rows, 1> ret;
    memcpy(ret.data(), glm::value_ptr(v), sizeof(T) * Rows);
    return ret;
}

template <class T, int Rows, int Cols>
inline glm::mat<Cols, Rows, T> eigen_to_glm(const Eigen::Matrix<T, Rows, Cols>& M) {
    glm::mat<Cols, Rows, T> ret;
    memcpy(glm::value_ptr(ret), M.data(), sizeof(T) * Rows * Cols);
    return ret;
}

template <class T, int Rows>
inline glm::vec<Rows, T> eigen_to_glm(const Eigen::Matrix<T, Rows, 1>& v) {
    glm::vec<Rows, T> ret;
    memcpy(glm::value_ptr(ret), v.data(), sizeof(T) * Rows);
    return ret;
}

inline glmx::dynmat_view<artsim::real> get_view(Eigen::MatrixXr& mat) {
    return glmx::dynmat_view<artsim::real>(mat.data(), mat.rows(), mat.cols());
}

inline Eigen::Vector3r glm_to_eigen(const glm::rvec3& v) {
    return {v.x, v.y, v.z};
}

inline Eigen::Matrix3r glm_to_eigen(const glm::tmat3x3<artsim::real>& M) {
    return Eigen::Map<Eigen::Matrix3r>((artsim::real*)&M[0], 3, 3).transpose();
}

inline Eigen::Matrix3r glm_to_eigen(const glmx::tsmat3x3<artsim::real>& M) {
    Eigen::Matrix3r Me;
    Me(0, 0) = M.xx; Me(1, 1) = M.yy; Me(2, 2) = M.zz;
    Me(1, 2) = Me(2, 1) = M.yz;
    Me(2, 0) = Me(0, 2) = M.zx;
    Me(0, 1) = Me(1, 0) = M.xy;
    return Me;
}

inline Eigen::Matrix<artsim::real, 6, 6> glm_to_eigen(const glmx::tsmat6x6<artsim::real>& I) {
    Eigen::Matrix<artsim::real, 6, 6> M;
    M.block<3,3>(0, 0) = glm_to_eigen(I.I);
    M.block<3,3>(3, 0) = glm_to_eigen(I.C);
    M.block<3,3>(0, 3) = glm_to_eigen(I.C).transpose();
    M.block<3,3>(3, 3) = glm_to_eigen(I.M);
    return M;
}

inline glm::rvec3 eigen_to_glm(const Eigen::Vector3r& v) {
    return {v(0), v(1), v(2)};
}

#endif //EOS_SCAN_TO_HUMAN_EIGEN_H
