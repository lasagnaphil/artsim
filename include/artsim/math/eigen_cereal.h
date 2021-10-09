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
typename std::enable_if<traits::is_output_serializable<BinaryData<typename Derived::Scalar>, Archive>::value, void>::type
save(Archive& ar, Eigen::PlainObjectBase<Derived> const& m) {
    typedef Eigen::PlainObjectBase<Derived> ArrT;
    if (ArrT::RowsAtCompileTime == Eigen::Dynamic) ar(m.rows());
    if (ArrT::ColsAtCompileTime == Eigen::Dynamic) ar(m.cols());
    ar(binary_data(m.data(), m.size() * sizeof(typename Derived::Scalar)));
}

template <class Archive, class Derived>
inline
typename std::enable_if<traits::is_input_serializable<BinaryData<typename Derived::Scalar>, Archive>::value, void>::type
load(Archive& ar, Eigen::PlainObjectBase<Derived>& m) {
    typedef Eigen::PlainObjectBase<Derived> ArrT;
    Eigen::Index rows = ArrT::RowsAtCompileTime, cols = ArrT::ColsAtCompileTime;
    if (rows == Eigen::Dynamic) ar(rows);
    if (cols == Eigen::Dynamic) ar(cols);
    m.resize(rows, cols);
    ar(binary_data(m.data(), static_cast<std::size_t>(rows * cols * sizeof(typename Derived::Scalar))));
}

// From: https://stackoverflow.com/questions/24593085/serializing-decomposed-matrix-from-eigen-sparselu-object
template <class Archive, class Derived>
inline
typename std::enable_if<traits::is_input_serializable<BinaryData<typename Derived::Scalar>, Archive>::value, void>::type
load(Archive& ar, Eigen::SparseCompressedBase<Derived>& m) {
    typedef Eigen::SparseCompressedBase<Derived> MatT;
    typedef typename MatT::StorageIndex Index;
    typedef typename MatT::Scalar Scalar;
    Index rows, cols, nnzs, outS, innS;
    rows = m.rows();
    cols = m.cols();
    nnzs = m.nonZeros();
    outS = m.outerSize();
    innS = m.innerSize();
    ar(rows, cols, nnzs, outS, innS);
    ar(binary_data(m.valuePtr(), sizeof(Scalar) * nnzs));
    ar(binary_data(m.outerIndexPtr(), sizeof(Index) * outS));
    ar(binary_data(m.innerIndexPtr(), sizeof(Index) * innS));
}

}


#endif //ARTSIM_EIGEN_CEREAL_H
