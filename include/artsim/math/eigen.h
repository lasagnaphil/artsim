//
// Created by lasagnaphil on 7/17/21.
//

#ifndef ARTSIM_EIGEN_H
#define ARTSIM_EIGEN_H

#include <Eigen/Dense>
#include <artsim/math/se3.h>
#include <artsim/math/dynmat.h>

namespace artsim {

using MatrixXr = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic>;
using VectorXr = Eigen::Matrix<real, Eigen::Dynamic, 1>;
using Vector3r = Eigen::Matrix<real, 3, 1>;

inline glmx::dynmat_view<real> get_view(MatrixXr& mat) {
    return glmx::dynmat_view<real>(mat.data(), mat.rows(), mat.cols());
}

inline Vector3r glm_to_eigen(const glm::rvec3& v) {
    return {v.x, v.y, v.z};
}

inline Eigen::Matrix<real, 3, 3> glm_to_eigen(const glm::tmat3x3<real>& M) {
    return Eigen::Map<Eigen::Matrix<real, 3, 3>>((real*)&M[0], 3, 3).transpose();
}

inline Eigen::Matrix<real, 3, 3> glm_to_eigen(const glmx::tsmat3x3<real>& M) {
    Eigen::Matrix<real, 3, 3> Me;
    Me(0, 0) = M.xx; Me(1, 1) = M.yy; Me(2, 2) = M.zz;
    Me(1, 2) = Me(2, 1) = M.yz;
    Me(2, 0) = Me(0, 2) = M.zx;
    Me(0, 1) = Me(1, 0) = M.xy;
    return Me;
}

inline Eigen::Matrix<real, 6, 6> glm_to_eigen(const glmx::tsmat6x6<real>& I) {
    Eigen::Matrix<real, 6, 6> M;
    M.block<3,3>(0, 0) = glm_to_eigen(I.I);
    M.block<3,3>(3, 0) = glm_to_eigen(I.C);
    M.block<3,3>(0, 3) = glm_to_eigen(I.C).transpose();
    M.block<3,3>(3, 3) = glm_to_eigen(I.M);
    return M;
}

inline glm::rvec3 eigen_to_glm(const Vector3r& v) {
    return {v(0), v(1), v(2)};
}


}

#endif //ARTSIM_EIGEN_H
