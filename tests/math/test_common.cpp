//
// Created by lasagnaphil on 20. 12. 12..
//

#include "doctest.h"
#include "artsim/math/common.h"
#include "artsim/math/se3.h"
#include "utils/test_utils.h"

using namespace glm;
using namespace artsim;

static std::random_device random_dev;
static std::default_random_engine engine(random_dev());
using real_t = float;

void compare_glm(const glm::tmat3x3<real_t>& m1, const glm::tmat3x3<real_t>& m2) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            INFO("Iteration (" << i << ", " << j << ")");
            CHECK(m1[i][j] == doctest::Approx(m2[i][j]).epsilon(1e-6));
        }
    }
}

TEST_CASE("Rx, Ry, Rz") {
    real_t theta = std::uniform_real_distribution<real_t>(-glm::pi<real_t>(), glm::pi<real_t>())(engine);

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
