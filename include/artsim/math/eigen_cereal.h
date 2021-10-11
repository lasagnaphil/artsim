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
    if (Rows == Eigen::Dynamic) ar(m.rows());
    if (Cols == Eigen::Dynamic) ar(m.cols());
    ar(cereal::binary_data(m.data(), m.size() * sizeof(Scalar)));
}

template <class Archive, class Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
inline
typename std::enable_if<cereal::traits::is_input_serializable<cereal::BinaryData<Scalar>, Archive>::value, void>::type
load(Archive& ar, Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& m) {
    int rows = Rows, cols = Cols;
    if (rows == Eigen::Dynamic) ar(rows);
    if (cols == Eigen::Dynamic) ar(cols);
    m.resize(rows, cols);
    ar(cereal::binary_data(m.data(), static_cast<std::size_t>(rows * cols * sizeof(Scalar))));
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
inline void save(Archive& ar, const Eigen::SparseMatrix<Scalar>& m) {
    typedef typename Eigen::SparseMatrix<Scalar>::StorageIndex Index;
    Index rows, cols, nnzs, outS, innS;
    rows = m.rows();
    cols = m.cols();
    nnzs = m.nonZeros();
    outS = m.outerSize();
    innS = m.innerSize();
    ar(rows, cols, nnzs, outS, innS);
    ar(cereal::binary_data(m.valuePtr(), sizeof(Scalar) * nnzs));
    ar(cereal::binary_data(m.outerIndexPtr(), sizeof(Index) * outS));
    ar(cereal::binary_data(m.innerIndexPtr(), sizeof(Index) * innS));
}

template <class Archive, class Scalar>
inline void load(Archive& ar, Eigen::SparseMatrix<Scalar>& m) {
    typedef typename Eigen::SparseMatrix<Scalar>::StorageIndex Index;
    Index rows, cols, nnzs, outS, innS;
    ar(rows, cols, nnzs, outS, innS);
    m.resize(rows, cols);
    m.makeCompressed();
    m.resizeNonZeros(nnzs);
    ar(cereal::binary_data(m.valuePtr(), sizeof(Scalar) * nnzs));
    ar(cereal::binary_data(m.outerIndexPtr(), sizeof(Index) * outS));
    ar(cereal::binary_data(m.innerIndexPtr(), sizeof(Index) * nnzs));
}

template <class Archive, class Derived>
inline void save(Archive& ar, const Eigen::SimplicialCholeskyBase<Derived>& ldlt) {
    ar(ldlt.getInternalMatrix());
    ar(ldlt.getInternalDiag());
    ar(ldlt.getInternalParent());
    ar(ldlt.getInternalNonZerosPerCol());
    ar(ldlt.getInternalP().indices());
    ar(ldlt.getInternalPinv().indices());
}

template <class Archive, class Derived>
inline void load(Archive& ar, Eigen::SimplicialCholeskyBase<Derived>& ldlt) {
    ar(ldlt.getInternalMatrix());
    ar(ldlt.getInternalDiag());
    ar(ldlt.getInternalParent());
    ar(ldlt.getInternalNonZerosPerCol());
    ar(ldlt.getInternalP().indices());
    ar(ldlt.getInternalPinv().indices());
}

}


#endif //ARTSIM_EIGEN_CEREAL_H
