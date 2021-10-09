//
// Created by lasagnaphil on 10/9/21.
//

#ifndef ARTSIM_EIGEN_CEREAL_H
#define ARTSIM_EIGEN_CEREAL_H

#include <cereal/cereal.hpp>
#include <cereal/archives/binary.hpp>
#include <Eigen/Dense>

namespace cereal {
template <class Archive, class Derived>
inline
void // typename std::enable_if<cereal::traits::is_output_serializable<cereal::BinaryData<typename Derived::Scalar>, Archive>::value, void>::type
save(Archive& ar, Eigen::PlainObjectBase<Derived> const& m) {
    typedef Eigen::PlainObjectBase<Derived> ArrT;
    if (ArrT::RowsAtCompileTime == Eigen::Dynamic) ar(m.rows());
    if (ArrT::ColsAtCompileTime == Eigen::Dynamic) ar(m.cols());
    ar(cereal::binary_data(m.data(), m.size() * sizeof(typename Derived::Scalar)));
}

template <class Archive, class Derived>
inline
void // typename std::enable_if<cereal::traits::is_input_serializable<cereal::BinaryData<typename Derived::Scalar>, Archive>::value, void>::type
load(Archive& ar, Eigen::PlainObjectBase<Derived>& m) {
    typedef Eigen::PlainObjectBase<Derived> ArrT;
    Eigen::Index rows = ArrT::RowsAtCompileTime, cols = ArrT::ColsAtCompileTime;
    if (rows == Eigen::Dynamic) ar(rows);
    if (cols == Eigen::Dynamic) ar(cols);
    m.resize(rows, cols);
    ar(cereal::binary_data(m.data(), static_cast<std::size_t>(rows * cols * sizeof(typename Derived::Scalar))));
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
    ar(cereal::binary_data(m.innerIndexPtr(), sizeof(Index) * innS));
}

template <class Archive, class Derived>
inline void save(Archive& ar, const Eigen::SimplicialCholeskyBase<Derived>& ldlt) {
    ar(ldlt.getInternalMatrix());
    ar(ldlt.getInternalParent());
    ar(ldlt.getInternalNonZerosPerCol());
    ar(ldlt.permutationP().indices());
    ar(ldlt.permutationPinv().indices());
}

template <class Archive, class Derived>
inline void load(Archive& ar, Eigen::SimplicialCholeskyBase<Derived>& ldlt) {
    ar(ldlt.getInternalMatrix());
    ar(ldlt.getInternalParent());
    ar(ldlt.getInternalNonZerosPerCol());
    ar(ldlt.permutationP().indices());
    ar(ldlt.permutationPinv().indices());
}

}


#endif //ARTSIM_EIGEN_CEREAL_H
