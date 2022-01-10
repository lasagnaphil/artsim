//
// Created by lasagnaphil on 2/21/21.
//

#ifndef ARTSIM_FASTSVD_H
#define ARTSIM_FASTSVD_H

#include <artsim/math/svd.h>

namespace glmx {

template <class T, int nsimd = 8>
void fastsvd_simd(const glm::tmat3x3<T>* A, SVD_mats<T>* out, const int jmax = nsimd);

template <class T>
void fastsvd(const glm::tmat3x3<T>* A, int A_count, SVD_mats<T>* out);

}

#endif //ARTSIM_FASTSVD_H
