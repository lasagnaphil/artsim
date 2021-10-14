//
// Created by lasagnaphil on 21. 10. 1..
//

#ifndef ARTSIM_MATH_CEREAL_H
#define ARTSIM_MATH_CEREAL_H

#include <artsim/math/box.h>
#include <artsim/math/se3.h>

namespace glm {
template<class Archive, class T>
void serialize(Archive& ar, tvec3<T>& v) {
    ar(v[0], v[1], v[2]);
}

template<class Archive, class T>
void serialize(Archive& ar, tvec4<T>& v) {
    ar(v[0], v[1], v[2], v[3]);
}

template<class Archive, class T>
void serialize(Archive& ar, tquat<T>& v) {
    ar(v[0], v[1], v[2], v[3]);
}
}

namespace glmx {
template<class Archive, class T, int N>
void serialize(Archive& ar, glmx::tbox<N, T>& box) {
    ar(box.lo, box.hi);
}
template<class Archive, class T>
void serialize(Archive& ar, glmx::ttransform<T>& trans) {
    ar(trans.v, trans.R[0], trans.R[1], trans.R[2]);
}
template<class Archive, class T>
void serialize(Archive& ar, glmx::tsmat3x3<T>& M) {
    ar(M.xx, M.yy, M.zz, M.yz, M.zx, M.xy);
}
template<class Archive, class T>
void serialize(Archive& ar, glmx::tspmat<T>& I) {
    ar(I.I, I.c, I.m);
}
}

#endif //ARTSIM_MATH_CEREAL_H
