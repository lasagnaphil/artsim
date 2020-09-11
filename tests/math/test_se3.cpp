//
// Created by lasagnaphil on 20. 9. 10..
//

#include "doctest.h"
#include "artsim/math/common.h"
#include "artsim/math/se3.h"

using namespace glm;
using namespace artsim;

TEST_CASE("transform inverse") {
    transform t(vec3(1, 2, 3), Rx(0.1f)*Ry(0.2f)*Rz(0.3f));
    transform t_inv = inverse(t);
    transform id1 = t * t_inv;
    transform id2 = t_inv * t;
            REQUIRE(id1.v.x == doctest::Approx(0));
            REQUIRE(id1.v.y == doctest::Approx(0));
            REQUIRE(id1.v.z == doctest::Approx(0));
            REQUIRE(id1.q.w == doctest::Approx(1));
            REQUIRE(id1.q.x == doctest::Approx(0));
            REQUIRE(id1.q.y == doctest::Approx(0));
            REQUIRE(id1.q.z == doctest::Approx(0));
            REQUIRE(id2.v.x == doctest::Approx(0));
            REQUIRE(id2.v.y == doctest::Approx(0));
            REQUIRE(id2.v.z == doctest::Approx(0));
            REQUIRE(id2.q.w == doctest::Approx(1));
            REQUIRE(id2.q.x == doctest::Approx(0));
            REQUIRE(id2.q.y == doctest::Approx(0));
            REQUIRE(id2.q.z == doctest::Approx(0));
}

TEST_CASE("transform mat4_cast") {
    transform t(vec3(1, 2, 3), Rx(0.1f)*Ry(0.2f)*Rz(0.3f));
    mat4 M = mat4_cast(t);
    quat Mq = quat_cast(mat3(vec3(M[0]), vec3(M[1]), vec3(M[2])));
            REQUIRE(t.v.x == doctest::Approx(M[3][0]));
            REQUIRE(t.v.y == doctest::Approx(M[3][1]));
            REQUIRE(t.v.z == doctest::Approx(M[3][2]));
            REQUIRE(t.q.w == doctest::Approx(Mq.w));
            REQUIRE(t.q.x == doctest::Approx(Mq.x));
            REQUIRE(t.q.y == doctest::Approx(Mq.y));
            REQUIRE(t.q.z == doctest::Approx(Mq.z));
}
