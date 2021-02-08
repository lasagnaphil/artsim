//
// Created by lasagnaphil on 21. 2. 8..
//

#ifndef ARTSIM_SOFT_BODY_H
#define ARTSIM_SOFT_BODY_H

#include <glm/vec3.hpp>
#include <vector>
#include <artsim/artsim.h>
#include <Eigen/Dense>

namespace artsim {

struct SoftBody {
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> triangles;
    std::vector<Eigen::Vector4i> tetrahedrons;

    std::vector<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> B_m;
    std::vector<double> W;
    Eigen::MatrixXd M;

    double density = 1000;

    void load_from_mesh(const char* filename);

    void setup();
};



}

#endif //ARTSIM_SOFT_BODY_H
