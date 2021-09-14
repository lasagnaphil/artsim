//
// Created by lasagnaphil on 5/16/21.
//

#ifndef ARTSIM_SIMD_H
#define ARTSIM_SIMD_H

#include <artsim/types.h>

#include <experimental/simd>

namespace stdx = std::experimental;

using isimd = stdx::native_simd<int>;
using fsimd = stdx::native_simd<float>;
using dsimd = stdx::native_simd<double>;

#define NUMERIC_LIMITS_XMACRO(_T) \
    FIELD_DECL(_T, int, digits) \
    FIELD_DECL(_T, int, max_digits10)              \
    FIELD_DECL(_T, bool, is_signed)                 \
    FIELD_DECL(_T, bool, is_integer)                \
    FIELD_DECL(_T, bool, is_exact)                  \
    FIELD_DECL(_T, int, radix)    \
    FIELD_DECL(_T, int, min_exponent)              \
    FIELD_DECL(_T, int, min_exponent10)              \
    FIELD_DECL(_T, int, max_exponent)              \
    FIELD_DECL(_T, int, max_exponent10)              \
    FIELD_DECL(_T, bool, has_infinity)             \
    FIELD_DECL(_T, bool, has_quiet_NaN)            \
    FIELD_DECL(_T, bool, has_signaling_NaN)            \
    FIELD_DECL(_T, float_denorm_style, has_denorm)            \
    FIELD_DECL(_T, bool, has_denorm_loss)            \
    FIELD_DECL(_T, bool, is_iec559)            \
    FIELD_DECL(_T, bool, is_bounded)            \
    FIELD_DECL(_T, bool, is_modulo)            \
    FIELD_DECL(_T, bool, traps)            \
    FIELD_DECL(_T, bool, tinyness_before)            \
    FIELD_DECL(_T, float_round_style, round_style) \
    METHOD_DECL(_T, min) \
    METHOD_DECL(_T, max) \
    METHOD_DECL(_T, lowest)       \
    METHOD_DECL(_T, epsilon)      \
    METHOD_DECL(_T, round_error)      \
    METHOD_DECL(_T, infinity)      \
    METHOD_DECL(_T, quiet_NaN)      \
    METHOD_DECL(_T, signaling_NaN)      \
    METHOD_DECL(_T, denorm_min)


template<>
struct std::numeric_limits<isimd>
{
    static constexpr bool is_specialized = true;

#define FIELD_DECL(_T, _TField, _Field) static constexpr _TField _Field = std::numeric_limits<_T>::_Field;
#define METHOD_DECL(_T, _Method) static constexpr isimd _Method() noexcept { return std::numeric_limits<_T>::_Method(); }
    NUMERIC_LIMITS_XMACRO(int)
#undef FIELD_DECL
#undef METHOD_DECL
};

template<>
struct std::numeric_limits<fsimd>
{
    static constexpr bool is_specialized = true;

#define FIELD_DECL(_T, _TField, _Field) static constexpr _TField _Field = std::numeric_limits<_T>::_Field;
#define METHOD_DECL(_T, _Method) static constexpr fsimd _Method() noexcept { return std::numeric_limits<_T>::_Method(); }
    NUMERIC_LIMITS_XMACRO(float)
#undef FIELD_DECL
#undef METHOD_DECL
};

template<>
struct std::numeric_limits<dsimd>
{
    static constexpr bool is_specialized = true;

#define FIELD_DECL(_T, _TField, _Field) static constexpr _TField _Field = std::numeric_limits<_T>::_Field;
#define METHOD_DECL(_T, _Method) static constexpr dsimd _Method() noexcept { return std::numeric_limits<_T>::_Method(); }
    NUMERIC_LIMITS_XMACRO(double)
#undef FIELD_DECL
#undef METHOD_DECL
};

namespace glmx {
template <class T>
void load_simd(const glm::tmat3x3<T>*__restrict A, OUT glm::tmat3x3<stdx::native_simd<T>>&__restrict A_simd) {
    constexpr int simd_width = stdx::native_simd<T>::size();
    std::array<T, simd_width> buf;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < simd_width; k++) buf[k] = A[k][i][j];
            A_simd[i][j].copy_from(buf.data(), stdx::element_aligned);
        }
    }
}

template <class T>
void load_simd(const glm::tvec3<T>*__restrict v, OUT glm::tvec3<stdx::native_simd<T>>&__restrict v_simd) {
    constexpr int simd_width = stdx::native_simd<T>::size();
    std::array<T, simd_width> buf;
    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < simd_width; k++) buf[k] = v[k][i];
        v_simd[i].copy_from(buf.data(), stdx::element_aligned);
    }
}

template <class T>
void store_simd(glm::tmat3x3<stdx::native_simd<T>>&__restrict A_simd, OUT glm::tmat3x3<T>*__restrict A) {
    constexpr int simd_width = stdx::native_simd<T>::size();
    std::array<T, simd_width> buf;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            A_simd[i][j].copy_to(buf.data(), stdx::element_aligned);
            for (int k = 0; k < simd_width; k++) A[k][i][j] = buf[k];
        }
    }
}

template <class T>
void store_simd(glm::tvec3<stdx::native_simd<T>>&__restrict v_simd, OUT glm::tvec3<T>*__restrict v) {
    constexpr int simd_width = stdx::native_simd<T>::size();
    std::array<T, simd_width> buf;
    for (int i = 0; i < 3; i++) {
        v_simd[i].copy_from(buf.data(), stdx::element_aligned);
        for (int k = 0; k < simd_width; k++) v[k][i] = buf[k];
    }
}
}

#endif //ARTSIM_SIMD_H
