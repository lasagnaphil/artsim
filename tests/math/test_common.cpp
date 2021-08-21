//
// Created by lasagnaphil on 20. 12. 12..
//

#include "doctest.h"
#include <artsim/math/common.h>
#include <utils/test_utils.h>

using namespace glm;

static std::random_device random_dev;
static std::default_random_engine engine(random_dev());
using real = float;

TEST_CASE("Rx, Ry, Rz") {
    real theta = std::uniform_real_distribution<real>(-glm::pi<real>(), glm::pi<real>())(engine);

    SUBCASE("Rx") {
        auto m1 = Rx(theta);
        auto m2 = glm::mat3_cast(glm::quat(glm::cos(theta/2), glm::sin(theta/2), 0, 0));
        compare_glm(m1, m2);
    }

    SUBCASE("Ry") {
        auto m1 = Ry(theta);
        auto m2 = glm::mat3_cast(glm::quat(glm::cos(theta/2), 0, glm::sin(theta/2), 0));
        compare_glm(m1, m2);
    }

    SUBCASE("Rz") {
        auto m1 = Rz(theta);
        auto m2 = glm::mat3_cast(glm::quat(glm::cos(theta/2), 0, 0, glm::sin(theta/2)));
        compare_glm(m1, m2);
    }

}
