//
// Created by lasagnaphil on 2/21/21.
//

#ifndef ARTSIM_FASTSVD_H
#define ARTSIM_FASTSVD_H

#include <artsim/math/svd.h>
#include <artsim/math/simd.h>

namespace glmx {

template <class T>
void fastsvd(const glm::tmat3x3<T>* A, int A_count, SVD_mats<T>* out, int num_threads = 1);

void fastsvd(const glm::tmat3x3<fsimd>& A, OUT glmx::SVD_mats<fsimd>& A_svd);

}

#endif //ARTSIM_FASTSVD_H
