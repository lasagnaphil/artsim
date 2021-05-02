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

    int buf_size = 8 * ((A_count + 1) / 8) + 8;
    float* buf = (float*)aligned_alloc(32, buf_size*30*sizeof(float));

    float* a11 = buf + 0*buf_size;
    float* a12 = buf + 1*buf_size;
    float* a13 = buf + 2*buf_size;
    float* a21 = buf + 3*buf_size;
    float* a22 = buf + 4*buf_size;
    float* a23 = buf + 5*buf_size;
    float* a31 = buf + 6*buf_size;
    float* a32 = buf + 7*buf_size;
    float* a33 = buf + 8*buf_size;

    float* u11 = buf + 9*buf_size;
    float* u12 = buf + 10*buf_size;
    float* u13 = buf + 11*buf_size;
    float* u21 = buf + 12*buf_size;
    float* u22 = buf + 13*buf_size;
    float* u23 = buf + 14*buf_size;
    float* u31 = buf + 15*buf_size;
    float* u32 = buf + 16*buf_size;
    float* u33 = buf + 17*buf_size;

    float* v11 = buf + 18*buf_size;
    float* v12 = buf + 19*buf_size;
    float* v13 = buf + 20*buf_size;
    float* v21 = buf + 21*buf_size;
    float* v22 = buf + 22*buf_size;
    float* v23 = buf + 23*buf_size;
    float* v31 = buf + 24*buf_size;
    float* v32 = buf + 25*buf_size;
    float* v33 = buf + 26*buf_size;

    float* sigma1 = buf + 27*buf_size;
    float* sigma2 = buf + 28*buf_size;
    float* sigma3 = buf + 29*buf_size;

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
    Singular_Value_Decomposition_Size_Specific_Helper<float> task(A_count,
                                                                  a11,a21,a31,a12,a22,a32,a13,a23,a33,
                                                                  u11,u21,u31,u12,u22,u32,u13,u23,u33,
                                                                  v11,v21,v31,v12,v22,v32,v13,v23,v33,
                                                                  sigma1,sigma2,sigma3);

    if (num_threads == 1) {
        task.Run();
    }
    else {
#pragma omp parallel for default(none) firstprivate(num_threads, A_count, task)
        for (int partition = 0; partition < num_threads; partition++) {
            int imin = (A_count / num_threads) * partition + std::min(A_count % num_threads, partition);
            int imax_plus_one =
                    (A_count / num_threads) * (partition + 1) + std::min(A_count % num_threads, partition + 1);
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

    // Deallocate memory
    delete[] buf;
}

template void fastsvd(const glm::tmat3x3<artsim::real>* A, int A_count, SVD_mats<artsim::real>* out, int num_threads);

}