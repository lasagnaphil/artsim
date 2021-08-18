//
// Created by lasagnaphil on 7/23/21.
//

#ifndef EOS_SCAN_TO_HUMAN_EIGEN_H
#define EOS_SCAN_TO_HUMAN_EIGEN_H

#include <artsim/types.h>
#include <glm/gtc/type_ptr.hpp>

namespace Eigen {

using MatrixXr = Eigen::Matrix<artsim::real, Eigen::Dynamic, Eigen::Dynamic>;
using VectorXr = Eigen::Matrix<artsim::real, Eigen::Dynamic, 1>;

using Matrix3r = Eigen::Matrix<artsim::real, 3, 3>;
using Vector3r = Eigen::Matrix<artsim::real, 3, 1>;

}

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

#endif //EOS_SCAN_TO_HUMAN_EIGEN_H
