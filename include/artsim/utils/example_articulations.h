//
// Created by Phillip Chang on 2020/09/26.
//

#ifndef ARTSIM_EXAMPLE_ARTICULATIONS_H
#define ARTSIM_EXAMPLE_ARTICULATIONS_H

#include <vector>
#include <artsim/artsim.h>
#include <artsim/dynamics.h>
#include <chrono>

namespace artsim {

namespace examples {

    ArticulatedBody create_single_pendulum_link(bool spherical,
                                                float density = 1000.0f, float l = 1.0f, float d = 0.1f);
    ArticulatedBody create_double_pendulum_ball(bool spherical, float m1 = 1.0f, float m2 = 1.0f, float l1 = 1.0f, float l2 = 1.0f);
    ArticulatedBody create_double_pendulum_link(bool spherical,
                                                float density = 1000.0f, float l1 = 1.0f, float l2 = 1.0f, float d = 0.1f);
    ArticulatedBody create_triple_pendulum_link(bool spherical,
                                                float density = 1000.0f, float l1 = 1.0f, float l2 = 1.0f, float l3 = 1.0f, float d = 0.1f);
    ArticulatedBody create_furuta_pendulum(bool spherical, float density = 1000.0f, float l1 = 1.0f, float l2 = 1.0f, float d = 0.1f);
    ArticulatedBody create_5_link_tree(bool spherical = false);
    ArticulatedBody create_13_link_tree(bool spherical = false);
    ArticulatedBody create_free_link(int num_links, bool spherical = false);
}

}

#endif //ARTSIM_EXAMPLE_ARTICULATIONS_H
