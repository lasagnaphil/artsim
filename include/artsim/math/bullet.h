//
// Created by lasagnaphil on 8/21/21.
//

#ifndef ARTSIM_BULLET_H
#define ARTSIM_BULLET_H

#include <artsim/types.h>

#include <LinearMath/btVector3.h>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btTransform.h>

namespace artsim {

inline btVector3 btconv(const glm::rvec3& v) {
    return btVector3(v.x, v.y, v.z);
}

inline btMatrix3x3 btconv(const glm::rmat3& M) {
    return btMatrix3x3(M[0][0], M[1][0], M[2][0], M[0][1], M[1][1], M[2][1], M[0][2], M[1][2], M[2][2]);
}

inline btQuaternion btconv(const glm::rquat& q) {
    return btQuaternion(q.x, q.y, q.z, q.w);
}

inline btTransform btconv(const glmx::rtransform& T) {
    return btTransform(btconv(T.R), btconv(T.v));
}

inline btTransform btconv(const glmx::rquat_transform& T) {
    return btTransform(btconv(T.q), btconv(T.v));
}

inline glm::rvec3 glmconv(const btVector3& v) {
    return {v.x(), v.y(), v.z()};
}

inline glm::rmat3 glmconv(const btMatrix3x3& M) {
    return {glmconv(M.getColumn(0)), glmconv(M.getColumn(1)), glmconv(M.getColumn(2))};
}

inline glmx::rtransform glmconv(const btTransform& T) {
    return {glmconv(T.getOrigin()), glmconv(T.getBasis())};
}
}

#endif //ARTSIM_BULLET_H
