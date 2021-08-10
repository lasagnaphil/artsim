//
// Created by lasagnaphil on 7/23/21.
//

#ifndef EOS_SCAN_TO_HUMAN_EIGEN_H
#define EOS_SCAN_TO_HUMAN_EIGEN_H

#include <artsim/types.h>

namespace Eigen {

using MatrixXr = Eigen::Matrix<artsim::real, Eigen::Dynamic, Eigen::Dynamic>;
using VectorXr = Eigen::Matrix<artsim::real, Eigen::Dynamic, 1>;

using Matrix3r = Eigen::Matrix<artsim::real, 3, 3>;
using Vector3r = Eigen::Matrix<artsim::real, 3, 1>;

}

inline Eigen::Vector3r glm_to_eigen(glm::rvec3 v) {
    return Eigen::Vector3r(v.x, v.y, v.z);
}

inline glm::rvec3 eigen_to_glm(const Eigen::Vector3r& v) {
    return {v(0), v(1), v(2)};
}

#endif //EOS_SCAN_TO_HUMAN_EIGEN_H
