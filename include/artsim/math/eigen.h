//
// Created by lasagnaphil on 7/17/21.
//

#ifndef ARTSIM_EIGEN_H
#define ARTSIM_EIGEN_H

#include <Eigen/Dense>

namespace artsim {

using MatrixXr = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic>;
using VectorXr = Eigen::Matrix<real, Eigen::Dynamic, 1>;

glmx::dynmat_view<real> get_view(MatrixXr& mat) {
    return glmx::dynmat_view<real>(mat.data(), mat.rows(), mat.cols());
}

}

#endif //ARTSIM_EIGEN_H
