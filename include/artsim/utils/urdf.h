//
// Created by lasagnaphil on 1/23/21.
//

#ifndef ARTSIM_URDF_H
#define ARTSIM_URDF_H

#include <artsim/artsim.h>
#include <tinyxml2.h>

namespace artsim {
void export_to_urdf(const ArticulatedBodySpec& art, const char* robot_name, const char* filename);
}

#endif //ARTSIM_URDF_H
