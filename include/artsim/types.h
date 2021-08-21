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
#define InputReal InputDouble
#else
using real = float;
constexpr real REAL_MAX = FLT_MAX;
#define ImGuiDataType_Real ImGuiDataType_Float
#define InputReal InputFloat
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

namespace artsim {

using rvec2 = glm::rvec2;
using rvec3 = glm::rvec3;
using rvec4 = glm::rvec4;
using rmat3 = glm::rmat3;
using rmat4 = glm::rmat4;
using rquat = glm::rquat;
using ivec2 = glm::ivec2;
using ivec3 = glm::ivec3;
using ivec4 = glm::ivec4;
using rtransform = glmx::rtransform;
using rquat_transform = glmx::rquat_transform;
using rscrew = glmx::rscrew;
using rsmat3x3 = glmx::rsmat3x3;
using rspmat = glmx::rspmat;
using rsmat6x6 = glmx::rsmat6x6;


}

#endif //ARTSIM_TYPES_H
