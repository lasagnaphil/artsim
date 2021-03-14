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

#define OUT
#define INOUT
#define rptr *__restrict

namespace artsim {


#ifdef ARTSIM_USE_DOUBLE
using real = double;
#else
using real = float;
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
#endif //ARTSIM_TYPES_H
