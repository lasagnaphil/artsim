//
// Created by lasagnaphil on 3/1/21.
//

#include <artsim/ik.h>
#include <Eigen/Dense>

using namespace artsim;
using namespace glmx;

void artsim::inverse_kinematics(
        const ArticulatedBody& art, uint32_t ee_idx, const ttransform<real>& ee_offset,
        glm::tvec3<real> ee_global_pos, INOUT artsim::real* q) {
    using namespace Eigen;
    using real = artsim::real;
    using MatrixXr = Matrix<real, Dynamic, Dynamic>;
    using VectorXr = Matrix<real, Dynamic, 1>;

    int num_joints = art.get_num_joints();
    int num_vel_dofs = art.get_num_vel_dofs();
    std::vector<tscrew<real>> S(num_vel_dofs);
    std::vector<tscrew<real>> S_ee(num_vel_dofs);
    std::vector<ttransform<real>> T_joint_global(num_joints);

    calc_S(art, q, S.data());

    Map<VectorXr> q_cur(q, num_vel_dofs);
    Matrix<real, 3, Dynamic> J_b(3, num_vel_dofs-6);

    VectorXr u = VectorXr::Zero(num_vel_dofs);

    glm::tvec3<real> pos_diff;
    const real epsilon = real(1e-6);
    int iter;
    for (iter = 0; iter < 100; iter++) {
        calc_transforms(art, q, T_joint_global.data(), nullptr);
        calc_body_jacobian(art, ee_idx, ttransform<real>(IDENTITY), S.data(), T_joint_global.data(), S_ee.data());
        auto T_global = T_joint_global[ee_idx] * ee_offset;
        for (int i = 6; i < num_vel_dofs; i++) {
            tvec3<real> v = T_global.R * S_ee[i].v;
            J_b(0, i-6) = v[0];
            J_b(1, i-6) = v[1];
            J_b(2, i-6) = v[2];
        }

        pos_diff = ee_global_pos - T_global.v;
        Matrix<real, 3, 1> dX((real*)&pos_diff);
        u.bottomRows(num_vel_dofs-6) = (J_b.transpose() * J_b).ldlt().solve(J_b.transpose() * dX);
        integrate_implicit_euler(art, 1.0f, nullptr, q_cur.data(), u.data());
        if (glm::length2(pos_diff) <= epsilon*epsilon) {
            break;
        }
    }
    std::cout << "IK convergence in " << iter << " iters." << std::endl;
}
