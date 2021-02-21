//
// Created by lasagnaphil on 2/21/21.
//

#include <artsim/math/fastsvd.h>
#include <fastsvd/fastsvd.h>
#include <artsim/types.h>

namespace glmx {

template <class T>
void fastsvd(const glm::tmat3x3<T>* A, int A_count, SVD_mats<T>* out, int num_threads) {
    using namespace Singular_Value_Decomposition;

    float *a11,*a21,*a31,*a12,*a22,*a32,*a13,*a23,*a33;
    float *u11,*u21,*u31,*u12,*u22,*u32,*u13,*u23,*u33;
    float *v11,*v21,*v31,*v12,*v22,*v32,*v13,*v23,*v33;
    float *sigma1,*sigma2,*sigma3;

    int size = A_count;

    // Allocate data
    Singular_Value_Decomposition_Size_Specific_Helper<float>::Allocate_Data(size + 8,
                                                                            a11,a21,a31,a12,a22,a32,a13,a23,a33,
                                                                            u11,u21,u31,u12,u22,u32,u13,u23,u33,
                                                                            v11,v21,v31,v12,v22,v32,v13,v23,v33,
                                                                            sigma1,sigma2,sigma3);

    // Insert data
    for (int i = 0; i < A_count; i++) {
        a11[i] = A[i][0][0];
        a21[i] = A[i][0][1];
        a31[i] = A[i][0][2];
        a12[i] = A[i][1][0];
        a22[i] = A[i][1][1];
        a32[i] = A[i][1][2];
        a13[i] = A[i][2][0];
        a23[i] = A[i][2][1];
        a33[i] = A[i][2][2];
    }

    // Run kernel
    Singular_Value_Decomposition_Size_Specific_Helper<float> task(size,
                                                                  a11,a21,a31,a12,a22,a32,a13,a23,a33,
                                                                  u11,u21,u31,u12,u22,u32,u13,u23,u33,
                                                                  v11,v21,v31,v12,v22,v32,v13,v23,v33,
                                                                  sigma1,sigma2,sigma3);

    if (num_threads == 1) {
        task.Run();
    }
    else {
#pragma omp parallel for default(none) firstprivate(num_threads, size, task)
        for (int partition = 0; partition < num_threads; partition++) {
            int imin = (size / num_threads) * partition + std::min(size % num_threads, partition);
            int imax_plus_one =
                    (size / num_threads) * (partition + 1) + std::min(size % num_threads, partition + 1);
            task.Run_Index_Range(imin, imax_plus_one);
        }
    }

    // Retrieve data
    for (int i = 0; i < A_count; i++) {
        out[i].U[0][0] = u11[i];
        out[i].U[0][1] = u21[i];
        out[i].U[0][2] = u31[i];
        out[i].U[1][0] = u12[i];
        out[i].U[1][1] = u22[i];
        out[i].U[1][2] = u32[i];
        out[i].U[2][0] = u13[i];
        out[i].U[2][1] = u23[i];
        out[i].U[2][2] = u33[i];

        out[i].V[0][0] = v11[i];
        out[i].V[0][1] = v21[i];
        out[i].V[0][2] = v31[i];
        out[i].V[1][0] = v12[i];
        out[i].V[1][1] = v22[i];
        out[i].V[1][2] = v32[i];
        out[i].V[2][0] = v13[i];
        out[i].V[2][1] = v23[i];
        out[i].V[2][2] = v33[i];

        out[i].Sigma[0] = sigma1[i];
        out[i].Sigma[1] = sigma2[i];
        out[i].Sigma[2] = sigma3[i];
    }
}

template void fastsvd(const glm::tmat3x3<artsim::real>* A, int A_count, SVD_mats<artsim::real>* out, int num_threads);

}