//
// Created by lasagnaphil on 2/16/21.
//

#ifndef ARTSIM_TYPES_H
#define ARTSIM_TYPES_H

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtx/quaternion.hpp>
#include <artsim/math/se3.h>

#define OUT
#define INOUT
#define rptr *__restrict

namespace artsim {


#ifdef ARTSIM_USE_DOUBLE
using real = double;
constexpr real REAL_MAX = DBL_MAX;
#define ImGuiDataType_Real ImGuiDataType_Double
#else
using real = float;
constexpr real REAL_MAX = FLT_MAX;
#define ImGuiDataType_Real ImGuiDataType_Float
#endif

}

namespace glm {

using rvec2 = glm::tvec2<artsim::real>;
using rvec3 = glm::tvec3<artsim::real>;
using rvec4 = glm::tvec4<artsim::real>;
using rmat3 = glm::tmat3x3<artsim::real>;
using rmat4 = glm::tmat4x4<artsim::real>;
using rquat = glm::tquat<artsim::real>;

}

namespace glmx {

using rtransform = glmx::ttransform<artsim::real>;
using rquat_transform = glmx::tquat_transform<artsim::real>;
using rscrew = glmx::tscrew<artsim::real>;
using rsmat3x3 = glmx::tsmat3x3<artsim::real>;
using rspmat = glmx::tspmat<artsim::real>;
using rsmat6x6 = glmx::tsmat6x6<artsim::real>;

}

#endif //ARTSIM_TYPES_H
