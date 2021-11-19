//
// Created by lasagnaphil on 10/9/21.
//

#ifndef ARTSIM_EIGEN_CEREAL_H
#define ARTSIM_EIGEN_CEREAL_H

#include <cereal/cereal.hpp>
#include <cereal/archives/binary.hpp>
#include <Eigen/Dense>

namespace cereal {
template <class Archive, class Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
inline
typename std::enable_if<cereal::traits::is_output_serializable<cereal::BinaryData<Scalar>, Archive>::value, void>::type
save(Archive& ar, const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& m) {
    int rows = m.rows(), cols = m.cols();
    ar(rows);
    ar(cols);
    ar(cereal::binary_data(m.data(), rows * cols * sizeof(Scalar)));
}

template <class Archive, class Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
inline
typename std::enable_if<cereal::traits::is_input_serializable<cereal::BinaryData<Scalar>, Archive>::value, void>::type
load(Archive& ar, Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& m) {
    int rows, cols;
    ar(rows);
    ar(cols);
    m.resize(rows, cols);
    ar(cereal::binary_data(m.data(), rows * cols * sizeof(Scalar)));
}

/*
template <class Archive, class Scalar, Eigen::Index Size>
inline void save(Archive& ar, const Eigen::Vector<Scalar, Size>& v) {
    if (Size == Eigen::Dynamic) ar(v.size());
    ar(binary_data(v.data(), v.size() * sizeof(Scalar)));
}

template <class Archive, class Scalar, Eigen::Index Size>
inline void load(Archive& ar, Eigen::Vector<Scalar, Size>& v) {
    if (Size == Eigen::Dynamic) {
        Eigen::Index size;
        ar(size);
        v.resize(size);
    }
    ar(binary_data(v.data(), v.size() * sizeof(Scalar)));
}
 */

// From: https://stackoverflow.com/questions/24593085/serializing-decomposed-matrix-from-eigen-sparselu-object

template <class Archive, class Scalar>
inline void save(Archive& ar, const Eigen::Triplet<Scalar>& m) {
    ar(m.row());
    ar(m.col());
    ar(m.value());
}

template <class Archive, class Scalar>
inline void load(Archive& ar, Eigen::Triplet<Scalar>& m) {
    int row, col;
    Scalar value;
    ar(row);
    ar(col);
    ar(value);
    m = Eigen::Triplet<Scalar>(row, col, value);
}

template <class Archive, class Scalar, int Options, class Index>
inline void save(Archive& ar, const Eigen::SparseMatrix<Scalar, Options, Index>& m) {
    int innerSize = m.innerSize();
    int outerSize = m.outerSize();
    using Triplet = Eigen::Triplet<Scalar>;
    std::vector<Triplet> triplets;
    for (int i = 0; i < outerSize; i++) {
        for (typename Eigen::SparseMatrix<Scalar, Options, Index>::InnerIterator it(m, i); it; ++it) {
            triplets.push_back(Triplet(it.row(), it.col(), it.value()));
        }
    }
    ar(innerSize);
    ar(outerSize);
    ar(triplets);
}

template <class Archive, class Scalar, int Options, class Index>
inline void load(Archive& ar, Eigen::SparseMatrix<Scalar, Options, Index>& m) {
    int innerSize, outerSize;
    ar(innerSize);
    ar(outerSize);
    int rows = m.IsRowMajor? outerSize : innerSize;
    int cols = m.IsRowMajor? innerSize : outerSize;
    m.resize(rows, cols);
    using Triplet = Eigen::Triplet<Scalar>;
    std::vector<Triplet> triplets;
    ar(triplets);
    m.setFromTriplets(triplets.begin(), triplets.end());
}

template <class Archive, class Derived>
inline void save(Archive& ar, const Eigen::SimplicialCholeskyBase<Derived>& ldlt) {
    ar(ldlt.getInternalMatrix());
    ar(ldlt.getInternalDiag());
    ar(ldlt.getInternalParent());
    ar(ldlt.getInternalNonZerosPerCol());
    ar(ldlt.getInternalP().indices());
    ar(ldlt.getInternalPinv().indices());
    ar(ldlt.getInternalInfo());
    ar(ldlt.getInternalFactorizationIsOk());
    ar(ldlt.getInternalAnalysisIsOk());
    ar(ldlt.getIsInitialized());
}

template <class Archive, class Derived>
inline void load(Archive& ar, Eigen::SimplicialCholeskyBase<Derived>& ldlt) {
    ar(ldlt.getInternalMatrix());
    ar(ldlt.getInternalDiag());
    ar(ldlt.getInternalParent());
    ar(ldlt.getInternalNonZerosPerCol());
    ar(ldlt.getInternalP().indices());
    ar(ldlt.getInternalPinv().indices());
    ar(ldlt.getInternalInfo());
    ar(ldlt.getInternalFactorizationIsOk());
    ar(ldlt.getInternalAnalysisIsOk());
    ar(ldlt.getIsInitialized());
}

}


#endif //ARTSIM_EIGEN_CEREAL_H
