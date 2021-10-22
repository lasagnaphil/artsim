//
// Created by lasagnaphil on 2/21/21.
//

#ifndef ARTSIM_FASTSVD_H
#define ARTSIM_FASTSVD_H

#include <artsim/math/svd.h>

#ifdef USE_SIMD
#include <artsim/math/simd.h>
#endif

namespace glmx {

template <class T>
void fastsvd(const glm::tmat3x3<T>* A, int A_count, SVD_mats<T>* out);

#ifdef USE_SIMD
void fastsvd(const glm::tmat3x3<fsimd>& A, OUT glmx::SVD_mats<fsimd>& A_svd);
#endif

}

#endif //ARTSIM_FASTSVD_H
