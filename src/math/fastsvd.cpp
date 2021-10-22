//
// Created by lasagnaphil on 2/21/21.
//

#include <artsim/types.h>
#include <artsim/math/fastsvd.h>
#include <fastsvd/fastsvd.h>
#include <tbb/parallel_for.h>
#include <tbb/enumerable_thread_specific.h>

#include <Tracy.hpp>

namespace glmx {

template <class T>
void fastsvd(const glm::tmat3x3<T>* A, int A_count, SVD_mats<T>* out) {
    ZoneScoped
    using namespace Singular_Value_Decomposition;

    constexpr int nsimd = 8;
    int N = (A_count - 1) / nsimd + 1;
    tbb::enumerable_thread_specific<std::array<float, 30*nsimd>> tls;
    // int buf_size = 30*nsimd*sizeof(float);
    // float* buf = (float*)aligned_alloc(nsimd*sizeof(float), buf_size);
    // memset(buf, 0, buf_size);

    tbb::parallel_for(size_t(0), size_t(N), [&](size_t i) {
        int jmax = glm::min<int>(A_count - nsimd*i, 8);
        auto buf = tls.local().data();
        float* a11 = buf + 0*nsimd;
        float* a12 = buf + 1*nsimd;
        float* a13 = buf + 2*nsimd;
        float* a21 = buf + 3*nsimd;
        float* a22 = buf + 4*nsimd;
        float* a23 = buf + 5*nsimd;
        float* a31 = buf + 6*nsimd;
        float* a32 = buf + 7*nsimd;
        float* a33 = buf + 8*nsimd;

        float* u11 = buf + 9 *nsimd;
        float* u12 = buf + 10*nsimd;
        float* u13 = buf + 11*nsimd;
        float* u21 = buf + 12*nsimd;
        float* u22 = buf + 13*nsimd;
        float* u23 = buf + 14*nsimd;
        float* u31 = buf + 15*nsimd;
        float* u32 = buf + 16*nsimd;
        float* u33 = buf + 17*nsimd;

        float* v11 = buf + 18*nsimd;
        float* v12 = buf + 19*nsimd;
        float* v13 = buf + 20*nsimd;
        float* v21 = buf + 21*nsimd;
        float* v22 = buf + 22*nsimd;
        float* v23 = buf + 23*nsimd;
        float* v31 = buf + 24*nsimd;
        float* v32 = buf + 25*nsimd;
        float* v33 = buf + 26*nsimd;

        float* sigma1 = buf + 27*nsimd;
        float* sigma2 = buf + 28*nsimd;
        float* sigma3 = buf + 29*nsimd;

        // Insert data
        for (int j = 0; j < jmax; j++) {
            a11[j] = A[nsimd*i+j][0][0];
            a21[j] = A[nsimd*i+j][0][1];
            a31[j] = A[nsimd*i+j][0][2];
            a12[j] = A[nsimd*i+j][1][0];
            a22[j] = A[nsimd*i+j][1][1];
            a32[j] = A[nsimd*i+j][1][2];
            a13[j] = A[nsimd*i+j][2][0];
            a23[j] = A[nsimd*i+j][2][1];
            a33[j] = A[nsimd*i+j][2][2];
        }

        // Run kernel
        Singular_Value_Decomposition_Size_Specific_Helper<float> task(nsimd,
                                                                      a11,a21,a31,a12,a22,a32,a13,a23,a33,
                                                                      u11,u21,u31,u12,u22,u32,u13,u23,u33,
                                                                      v11,v21,v31,v12,v22,v32,v13,v23,v33,
                                                                      sigma1,sigma2,sigma3);
        task.Run();

        for (int j = 0; j < jmax; j++) {
            out[nsimd*i+j].U[0][0] = u11[j];
            out[nsimd*i+j].U[0][1] = u21[j];
            out[nsimd*i+j].U[0][2] = u31[j];
            out[nsimd*i+j].U[1][0] = u12[j];
            out[nsimd*i+j].U[1][1] = u22[j];
            out[nsimd*i+j].U[1][2] = u32[j];
            out[nsimd*i+j].U[2][0] = u13[j];
            out[nsimd*i+j].U[2][1] = u23[j];
            out[nsimd*i+j].U[2][2] = u33[j];

            out[nsimd*i+j].V[0][0] = v11[j];
            out[nsimd*i+j].V[0][1] = v21[j];
            out[nsimd*i+j].V[0][2] = v31[j];
            out[nsimd*i+j].V[1][0] = v12[j];
            out[nsimd*i+j].V[1][1] = v22[j];
            out[nsimd*i+j].V[1][2] = v32[j];
            out[nsimd*i+j].V[2][0] = v13[j];
            out[nsimd*i+j].V[2][1] = v23[j];
            out[nsimd*i+j].V[2][2] = v33[j];

            out[nsimd*i+j].Sigma[0] = sigma1[j];
            out[nsimd*i+j].Sigma[1] = sigma2[j];
            out[nsimd*i+j].Sigma[2] = sigma3[j];
        }
    });
}

#ifdef USE_SIMD
void fastsvd(const glm::tmat3x3<fsimd>& A, OUT SVD_mats<fsimd>& A_svd) {
    ZoneScoped
    using namespace Singular_Value_Decomposition;

    float* a11 = (float*)&A[0][0];
    float* a21 = (float*)&A[0][1];
    float* a31 = (float*)&A[0][2];
    float* a12 = (float*)&A[1][0];
    float* a22 = (float*)&A[1][1];
    float* a32 = (float*)&A[1][2];
    float* a13 = (float*)&A[2][0];
    float* a23 = (float*)&A[2][1];
    float* a33 = (float*)&A[2][2];

    float* u11 = (float*)&A_svd.U[0][0];
    float* u21 = (float*)&A_svd.U[0][1];
    float* u31 = (float*)&A_svd.U[0][2];
    float* u12 = (float*)&A_svd.U[1][0];
    float* u22 = (float*)&A_svd.U[1][1];
    float* u32 = (float*)&A_svd.U[1][2];
    float* u13 = (float*)&A_svd.U[2][0];
    float* u23 = (float*)&A_svd.U[2][1];
    float* u33 = (float*)&A_svd.U[2][2];

    float* v11 = (float*)&A_svd.V[0][0];
    float* v21 = (float*)&A_svd.V[0][1];
    float* v31 = (float*)&A_svd.V[0][2];
    float* v12 = (float*)&A_svd.V[1][0];
    float* v22 = (float*)&A_svd.V[1][1];
    float* v32 = (float*)&A_svd.V[1][2];
    float* v13 = (float*)&A_svd.V[2][0];
    float* v23 = (float*)&A_svd.V[2][1];
    float* v33 = (float*)&A_svd.V[2][2];

    float* sigma1 = (float*)&A_svd.Sigma[0];
    float* sigma2 = (float*)&A_svd.Sigma[1];
    float* sigma3 = (float*)&A_svd.Sigma[2];

    Singular_Value_Decomposition_Size_Specific_Helper<float> task(fsimd::size(),
                                                                  a11,a21,a31,a12,a22,a32,a13,a23,a33,
                                                                  u11,u21,u31,u12,u22,u32,u13,u23,u33,
                                                                  v11,v21,v31,v12,v22,v32,v13,v23,v33,
                                                                  sigma1,sigma2,sigma3);

    task.Run();
}
#endif

template void fastsvd(const glm::tmat3x3<artsim::real>* A, int A_count, SVD_mats<artsim::real>* out);

}