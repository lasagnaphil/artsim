//
// Created by Phillip Chang on 2020/09/12.
//

#include "artsim/art_dynamics.h"

#include <Eigen/Dense>

using namespace glm;
using namespace glmx;

namespace artsim {

void set_zero_pose(const ArticulatedBodySpec& art, OUT real* q) {
    for (int i = 0; i < art.get_num_joints(); i++) {
        const Joint& joint = art.joints[i];
        const Link& link = art.links[i];
        uint32_t j = art.joint_pos_dof_starts[i];
        switch (joint.type) {
            case JOINT_TYPE_FLOATING: {
                q[0] = q[1] = q[2] = 0;
                q[3] = 0; q[4] = 0; q[5] = 0; q[6] = 1;
            } break;
            JOINT_DOF_1_CASE {
                q[j] = 0;
            } break;
            case JOINT_TYPE_SPHERICAL: {
                q[j] = 0; q[j+1] = 0; q[j+2] = 0; q[j+3] = 1;
            } break;
        }
    }
}

void calc_S(const ArticulatedBodySpec &art, const real *q, tscrew<real> *S) {
    for (int i = 0; i < art.get_num_joints(); i++) {
        const Joint& joint = art.joints[i];
        const Link& link = art.links[i];
        uint32_t vel_start = art.joint_vel_dof_starts[i];
        uint32_t vel_dof = art.joint_vel_dofs[i];
        switch (joint.type) {
            case JOINT_TYPE_FLOATING: {
                S[vel_start + 0] = tscrew<real>(1, 0, 0, 0, 0, 0);
                S[vel_start + 1] = tscrew<real>(0, 1, 0, 0, 0, 0);
                S[vel_start + 2] = tscrew<real>(0, 0, 1, 0, 0, 0);
                S[vel_start + 3] = tscrew<real>(0, 0, 0, 1, 0, 0);
                S[vel_start + 4] = tscrew<real>(0, 0, 0, 0, 1, 0);
                S[vel_start + 5] = tscrew<real>(0, 0, 0, 0, 0, 1);
            } break;
            case JOINT_TYPE_REVOLUTE_X: S[vel_start] = tscrew<real>(1, 0, 0, 0, 0, 0); break;
            case JOINT_TYPE_REVOLUTE_Y: S[vel_start] = tscrew<real>(0, 1, 0, 0, 0, 0); break;
            case JOINT_TYPE_REVOLUTE_Z: S[vel_start] = tscrew<real>(0, 0, 1, 0, 0, 0); break;
            case JOINT_TYPE_PRISMATIC_X: S[vel_start] = tscrew<real>(0, 0, 0, 1, 0, 0); break;
            case JOINT_TYPE_PRISMATIC_Y: S[vel_start] = tscrew<real>(0, 0, 0, 0, 1, 0); break;
            case JOINT_TYPE_PRISMATIC_Z: S[vel_start] = tscrew<real>(0, 0, 0, 0, 0, 1); break;
            case JOINT_TYPE_SPHERICAL: {
                S[vel_start + 0] = tscrew<real>(1, 0, 0, 0, 0, 0);
                S[vel_start + 1] = tscrew<real>(0, 1, 0, 0, 0, 0);
                S[vel_start + 2] = tscrew<real>(0, 0, 1, 0, 0, 0);
            } break;
        }
    }
}

ttransform<real> calc_Tinv(const Joint& joint, const Link& link, const real* q) {
    auto inv_linkT = ttransform<real>(inverse(link.local_joint_pose));
    switch (joint.type) {
        case JOINT_TYPE_FLOATING: return inverse(rtransform(make_vec3(q), mat3_cast(make_quat(q)))) * inv_linkT;
        case JOINT_TYPE_REVOLUTE_X: return Rx(-q[0]) * inv_linkT;
        case JOINT_TYPE_REVOLUTE_Y: return Ry(-q[0]) * inv_linkT;
        case JOINT_TYPE_REVOLUTE_Z: return Rz(-q[0]) * inv_linkT;
        case JOINT_TYPE_PRISMATIC_X: return tvec3<real>(-q[0], 0, 0) * inv_linkT;
        case JOINT_TYPE_PRISMATIC_Y: return tvec3<real>(0, -q[0], 0) * inv_linkT;
        case JOINT_TYPE_PRISMATIC_Z: return tvec3<real>(0, 0, -q[0]) * inv_linkT;
        case JOINT_TYPE_SPHERICAL: {
            glm::tquat<real> q_inv = glm::inverse(glm::make_quat<real>(q));
            return mat3_cast<real>(q_inv) * inv_linkT;
        }
        default: return ttransform<real>(IDENTITY);
    }
}

tscrew<real> calc_v0(const Joint& joint, const real* u) {
    switch (joint.type) {
        case JOINT_TYPE_FLOATING: return make_tscrew(u);
        case JOINT_TYPE_REVOLUTE_X: return tscrew<real>(u[0], 0, 0, 0, 0, 0);
        case JOINT_TYPE_REVOLUTE_Y: return tscrew<real>(0, u[0], 0, 0, 0, 0);
        case JOINT_TYPE_REVOLUTE_Z: return tscrew<real>(0, 0, u[0], 0, 0, 0);
        case JOINT_TYPE_PRISMATIC_X: return tscrew<real>(0, 0, 0, u[0], 0, 0);
        case JOINT_TYPE_PRISMATIC_Y: return tscrew<real>(0, 0, 0, 0, u[0], 0);
        case JOINT_TYPE_PRISMATIC_Z: return tscrew<real>(0, 0, 0, 0, 0, u[0]);
        case JOINT_TYPE_SPHERICAL: return tscrew<real>(u[0], u[1], u[2], 0, 0, 0);
        default: return tscrew<real>(IDENTITY);
    }
}

void calc_body_jacobian(const ArticulatedBodySpec& art, uint32_t joint_idx, const ttransform<real>& offset,
                        const ttransform<real>* T_joint_global,
                        tscrew<real>* J_b) {
    std::fill_n(J_b, art.get_num_vel_dofs(), tscrew<real>(IDENTITY));
    auto T_m = T_joint_global[joint_idx] * offset;
    int i = joint_idx;
    do {
        int joint_vel_dof_start = art.joint_vel_dof_starts[i];
        int joint_vel_dofs = art.joint_vel_dofs[i];
        auto& joint = art.joints[i];
        auto T = T_joint_global[i] / T_m;
        // TODO: inline this further (create specialized Ad functions)
        switch (joint.type) {
            case JOINT_TYPE_REVOLUTE_X:  J_b[joint_vel_dof_start] = {T.R[0], glm::cross(T.v, T.R[0])}; break;
            case JOINT_TYPE_REVOLUTE_Y:  J_b[joint_vel_dof_start] = {T.R[1], glm::cross(T.v, T.R[1])}; break;
            case JOINT_TYPE_REVOLUTE_Z:  J_b[joint_vel_dof_start] = {T.R[2], glm::cross(T.v, T.R[2])}; break;
            case JOINT_TYPE_PRISMATIC_X: J_b[joint_vel_dof_start] = {glm::rvec3(0), T.R[0]}; break;
            case JOINT_TYPE_PRISMATIC_Y: J_b[joint_vel_dof_start] = {glm::rvec3(0), T.R[1]}; break;
            case JOINT_TYPE_PRISMATIC_Z: J_b[joint_vel_dof_start] = {glm::rvec3(0), T.R[2]}; break;
            case JOINT_TYPE_SPHERICAL: {
                J_b[joint_vel_dof_start+0] = {T.R[0], glm::cross(T.v, T.R[0])};
                J_b[joint_vel_dof_start+1] = {T.R[1], glm::cross(T.v, T.R[1])};
                J_b[joint_vel_dof_start+2] = {T.R[2], glm::cross(T.v, T.R[2])};
            } break;
            case JOINT_TYPE_FLOATING: {
                J_b[joint_vel_dof_start+0] = {T.R[0], glm::cross(T.v, T.R[0])};
                J_b[joint_vel_dof_start+1] = {T.R[1], glm::cross(T.v, T.R[1])};
                J_b[joint_vel_dof_start+2] = {T.R[2], glm::cross(T.v, T.R[2])};
                J_b[joint_vel_dof_start+3] = {glm::rvec3(0), T.R[0]};
                J_b[joint_vel_dof_start+4] = {glm::rvec3(0), T.R[1]};
                J_b[joint_vel_dof_start+5] = {glm::rvec3(0), T.R[2]};
            } break;
        }
        i = art.parents[i];
    } while (i != -1);
}

void calc_linear_jacobian(const ArticulatedBodySpec& art, uint32_t joint_idx, const rtransform& offset,
                          const rtransform* T_joint_global,
                          OUT dynmat_view<real> Jc) {
    assert(Jc.rows == 3);
    assert(Jc.cols == art.get_num_vel_dofs());
    Jc.clear_zero();
    auto T_m = T_joint_global[joint_idx] * offset;
    int i = joint_idx;
    do {
        int joint_vel_dof_start = art.joint_vel_dof_starts[i];
        int joint_vel_dofs = art.joint_vel_dofs[i];
        auto& joint = art.joints[i];
        auto T = T_joint_global[i] / T_m;
        auto set_jacobian = [&Jc, joint_vel_dof_start](int i, glm::rvec3 v) {
            Jc(0, joint_vel_dof_start+i) = v[0];
            Jc(1, joint_vel_dof_start+i) = v[1];
            Jc(2, joint_vel_dof_start+i) = v[2];
        };
        switch (joint.type) {
            case JOINT_TYPE_REVOLUTE_X: set_jacobian(0, glm::cross(T.v, T.R[0])); break;
            case JOINT_TYPE_REVOLUTE_Y: set_jacobian(0, glm::cross(T.v, T.R[1])); break;
            case JOINT_TYPE_REVOLUTE_Z: set_jacobian(0, glm::cross(T.v, T.R[2])); break;
            case JOINT_TYPE_PRISMATIC_X: set_jacobian(0, T.R[0]); break;
            case JOINT_TYPE_PRISMATIC_Y: set_jacobian(0, T.R[1]); break;
            case JOINT_TYPE_PRISMATIC_Z: set_jacobian(0, T.R[2]); break;
            case JOINT_TYPE_SPHERICAL: {
                set_jacobian(0, glm::cross(T.v, T.R[0]));
                set_jacobian(1, glm::cross(T.v, T.R[1]));
                set_jacobian(2, glm::cross(T.v, T.R[2]));
            } break;
            case JOINT_TYPE_FLOATING: {
                set_jacobian(0, glm::cross(T.v, T.R[0]));
                set_jacobian(1, glm::cross(T.v, T.R[1]));
                set_jacobian(2, glm::cross(T.v, T.R[2]));
                set_jacobian(3, T.R[0]);
                set_jacobian(4, T.R[1]);
                set_jacobian(5, T.R[2]);
            } break;
        }
        i = art.parents[i];
    } while (i != -1);
}

void calc_linear_jacobian_transpose(const ArticulatedBodySpec& art, uint32_t joint_idx, const rtransform& offset,
                                    const rtransform* T_joint_global,
                                    OUT dynmat_view<real> Jc_T) {
    assert(Jc_T.rows == art.get_num_vel_dofs());
    assert(Jc_T.cols == 3);
    Jc_T.clear_zero();
    auto T_m = T_joint_global[joint_idx] * offset;
    int i = joint_idx;
    do {
        int joint_vel_dof_start = art.joint_vel_dof_starts[i];
        int joint_vel_dofs = art.joint_vel_dofs[i];
        auto& joint = art.joints[i];
        auto T = T_joint_global[i] / T_m;
        auto set_jacobian = [&Jc_T, joint_vel_dof_start](int i, glm::rvec3 v) {
            Jc_T(joint_vel_dof_start+i, 0) = v[0];
            Jc_T(joint_vel_dof_start+i, 1) = v[1];
            Jc_T(joint_vel_dof_start+i, 2) = v[2];
        };
        switch (joint.type) {
            case JOINT_TYPE_REVOLUTE_X: set_jacobian(0, glm::cross(T.v, T.R[0])); break;
            case JOINT_TYPE_REVOLUTE_Y: set_jacobian(0, glm::cross(T.v, T.R[1])); break;
            case JOINT_TYPE_REVOLUTE_Z: set_jacobian(0, glm::cross(T.v, T.R[2])); break;
            case JOINT_TYPE_PRISMATIC_X: set_jacobian(0, T.R[0]); break;
            case JOINT_TYPE_PRISMATIC_Y: set_jacobian(0, T.R[1]); break;
            case JOINT_TYPE_PRISMATIC_Z: set_jacobian(0, T.R[2]); break;
            case JOINT_TYPE_SPHERICAL: {
                set_jacobian(0, glm::cross(T.v, T.R[0]));
                set_jacobian(1, glm::cross(T.v, T.R[1]));
                set_jacobian(2, glm::cross(T.v, T.R[2]));
            } break;
            case JOINT_TYPE_FLOATING: {
                set_jacobian(0, glm::cross(T.v, T.R[0]));
                set_jacobian(1, glm::cross(T.v, T.R[1]));
                set_jacobian(2, glm::cross(T.v, T.R[2]));
                set_jacobian(3, T.R[0]);
                set_jacobian(4, T.R[1]);
                set_jacobian(5, T.R[2]);
            } break;
        }
        i = art.parents[i];
    } while (i != -1);
}

struct RecursiveNewtonEulerData {
    // IN
    bool is_floating_art;
    JointType joint_type;
    bool has_parent;

    ttransform<real> Tinv;
    tscrew<real> v0;
    tspmat<real> I;
    tscrew<real> f_ext;
    tvec3<real> udot;         // dof
    real kd;

    // INTERMEDIATE VALUES
    // ttransform<real> T_global_inv;

    // OUT
    tscrew<real> v;
    tscrew<real> a;
    tscrew<real> f;
    tvec3<real> tau;           // dof

    // kin must be calculated using jcalc() before this call
    void rnea_pass1() {
        v = Ad(Tinv, v) + v0;
        a = Ad(Tinv, a) + ad(v, v0); // + c0; (c0 is zero for all types of joints)
        if (!is_floating_art) {
            switch (joint_type) {
                JOINT_DOF_1_CASE {
                    int k = get_screw_idx(joint_type);
                    a[k] += udot[0];
                } break;
                case JOINT_TYPE_SPHERICAL: {
                    a.w += udot;
                } break;
            }
        }
        f = I * a - adT(v, I * v) - f_ext;
    }

    void rnea_pass2(real dt) {
        switch (joint_type) {
            JOINT_DOF_1_CASE {
                int k = get_screw_idx(joint_type);
                tau[0] = f[k] + kd * (v0[k] + dt * udot[0]);
            } break;
            case JOINT_TYPE_SPHERICAL: {
                tau = f.w + kd * (v0.w + dt * udot);
            } break;
        }
        if (has_parent) {
            f = AdT(Tinv, f);
        }
    }
};

void rne_inverse_dynamics(const ArticulatedBodySpec& art, glm::tvec3<real> gravity, real dt,
                          const real* q, const real* u, const real* udot, const tscrew<real>* f_ext,
                          real* tau) {

    int num_joints = art.get_num_joints();
    auto data = new RecursiveNewtonEulerData[num_joints];

    for (int i = 0; i < num_joints; i++) {
        auto& joint = art.joints[i];
        auto& link = art.links[i];
        uint32_t cur_pos_dof = art.joint_pos_dof_starts[i];
        uint32_t cur_vel_dof = art.joint_vel_dof_starts[i];
        int num_vel_dofs = art.joint_vel_dofs[i];
        data[i].is_floating_art = art.floating;
        data[i].joint_type = joint.type;
        data[i].has_parent = i != 0;
        data[i].Tinv = calc_Tinv(joint, link, q + cur_pos_dof);
        data[i].v0 = calc_v0(joint, u + cur_vel_dof);
        data[i].I = link.I_j;
        if (f_ext) data[i].f_ext = f_ext[i];
        if (!art.floating || i != 0) {
            for (int j = 0; j < num_vel_dofs; j++) {
                data[i].udot[j] = udot[cur_vel_dof + j];
            }
        }
        data[i].kd = joint.kd;
    }

    for (int i : art.bfs_iteration_order) {
        if (i == 0) {
            if (art.floating) {
                ttransform<real> T_root = ttransform<real>(make_vec3(q), glm::mat3_cast(make_quat(q + 3)));
                data[0].v = make_tscrew(u);
                data[0].a = Ad(inverse(T_root), tscrew<real>(tvec3<real>(0), -gravity));
                data[0].f = data[0].I * data[0].a - adT(data[0].v, data[0].I * data[0].v) - data[0].f_ext;
                continue;
            }
            else {
                data[0].v = tscrew<real>(IDENTITY);
                data[0].a = tscrew<real>(tvec3<real>(0), -gravity);
            }
        }
        else {
            data[i].v = data[art.parents[i]].v;
            data[i].a = data[art.parents[i]].a;
        }
        data[i].rnea_pass1();
    }

    int j_limit = art.floating? 1 : 0;
    for (int j = num_joints - 1; j >= j_limit; j--) {
        int i = art.bfs_iteration_order[j];
        data[i].rnea_pass2(dt);
        if (i != 0) {
            data[art.parents[i]].f += data[i].f;
        }
    }

    for (int i = 0; i < num_joints; i++) {
        uint32_t cur_vel_dof = art.joint_vel_dof_starts[i];
        int num_vel_dofs = art.joint_vel_dofs[i];
        if (art.joints[i].type == JOINT_TYPE_FLOATING) {
            tau[0] = data[0].f.w[0];
            tau[1] = data[0].f.w[1];
            tau[2] = data[0].f.w[2];
            tau[3] = data[0].f.v[0];
            tau[4] = data[0].f.v[1];
            tau[5] = data[0].f.v[2];
        }
        else {
            for (int j = 0; j < num_vel_dofs; j++) {
                tau[cur_vel_dof + j] = data[i].tau[j];
            }
        }
    }

    delete [] data;
}

struct FeatherstoneData {
    JointType joint_type;
    bool has_parent;

    // IN
    ttransform<real> Tinv;
    tscrew<real> v0;
    tscrew<real> f_ext;
    tvec3<real> tau;          // dof
    real kd;

    // INTERMEDIATE VALUES
    // ttransform<real> T_global_inv;
    tsmat6x6<real> I_a;
    tscrew<real> p_a;
    tscrew<real> v;
    tscrew<real> c;
    union {
        struct {
            tscrew<real> D;
            real H;
        } j1dof;
        struct {
            tsmat3x3<real> Dinv;
            tsmat3x3<real> I_Dinv;
            tmat3x3<real> Ct_Dinv;
        } j3dof;
    };
    tvec3<real> u; // dof
    tscrew<real> a;

    // OUT
    tvec3<real> udot;         // dof

    // kin must be calculated using jcalc() before this call
    inline void forward_pass1() {
        v = Ad(Tinv, v) + v0;
        c = ad(v, v0); // + c0; (c0 is zero for all joints)
        p_a = -adT(v, I_a * v) - f_ext;
    }

    inline void forward_pass2() {
        tscrew<real> a_p = Ad(Tinv, a) + c;
        switch (joint_type) {
            JOINT_DOF_1_CASE {
                int k = get_screw_idx(joint_type);
                udot[0] = (u[0] - dot(j1dof.D, a_p)) / j1dof.H;
                a = a_p;
                a[k] += udot[0];
            } break;
            case JOINT_TYPE_SPHERICAL: {
                udot = j3dof.Dinv * u - j3dof.I_Dinv * a_p.w - glm::transpose(j3dof.Ct_Dinv) * a_p.v;
                a = a_p;
                a.w += udot;
            } break;
        }
    }

    template <bool has_parent>
    inline void backward_pass(real dt) {
        switch (joint_type) {
            JOINT_DOF_1_CASE {
                int k = get_screw_idx(joint_type);
                j1dof.D = I_a[k];
                j1dof.H = j1dof.D[k] + kd * dt;
                u[0] = tau[0] - p_a[k];
                if constexpr (has_parent) {
                    tsmat6x6<real> I_prime = I_a - symmetric_cartesian_product(j1dof.D) / j1dof.H;
                    tscrew<real> p_prime = p_a + I_prime * c + ((u[0] - kd * v0[k]) / j1dof.H) * j1dof.D;
                    I_a = inv_transform(I_prime, Tinv);
                    p_a = AdT(Tinv, p_prime);
                }
            } break;
            case JOINT_TYPE_SPHERICAL: {
                j3dof.Dinv = inverse(I_a.I + tsmat3x3<real>(kd * dt));
                j3dof.I_Dinv = I_a.I * j3dof.Dinv;
                j3dof.Ct_Dinv = glm::transpose(I_a.C) * mat3_cast(j3dof.Dinv);
                u = tau - p_a.w;
                if constexpr (has_parent) {
                    auto I_prime = tsmat6x6<real>(
                            I_a.I - j3dof.I_Dinv * I_a.I,
                            I_a.C - mat3_cast(j3dof.I_Dinv) * I_a.C,
                            I_a.M - smat3_cast(j3dof.Ct_Dinv * I_a.C));
                    tvec3<real> tau_prime = u - kd * v0.w;
                    tscrew<real> p_prime = p_a;
                    p_prime += I_prime * c;
                    p_prime.w += j3dof.I_Dinv * tau_prime;
                    p_prime.v += j3dof.Ct_Dinv * tau_prime;
                    I_a = inv_transform(I_prime, Tinv);
                    p_a = AdT(Tinv, p_prime);
                }
            } break;
        }
    }


    template <bool has_parent>
    inline void invmass_backward_pass1(real dt) {
        switch (joint_type) {
            JOINT_DOF_1_CASE {
                int k = get_screw_idx(joint_type);
                j1dof.D = I_a[k];
                j1dof.H = j1dof.D[k] + kd * dt;
                if constexpr (has_parent) {
                    tsmat6x6<real> I_prime = I_a - symmetric_cartesian_product(j1dof.D) / j1dof.H;
                    I_a = inv_transform(I_prime, Tinv);
                }
            } break;
            case JOINT_TYPE_SPHERICAL: {
                j3dof.Dinv = inverse(I_a.I + tsmat3x3<real>(kd * dt));
                j3dof.I_Dinv = I_a.I * j3dof.Dinv;
                j3dof.Ct_Dinv = glm::transpose(I_a.C) * mat3_cast(j3dof.Dinv);
                if constexpr (has_parent) {
                    auto I_prime = tsmat6x6<real>(
                            I_a.I - j3dof.I_Dinv * I_a.I,
                            I_a.C - mat3_cast(j3dof.I_Dinv) * I_a.C,
                            I_a.M - smat3_cast(j3dof.Ct_Dinv * I_a.C));
                    I_a = inv_transform(I_prime, Tinv);
                }
            } break;
        }
    }

    template <bool has_parent>
    inline void invmass_backward_pass2() {
        switch (joint_type) {
            JOINT_DOF_1_CASE {
                int k = get_screw_idx(joint_type);
                u[0] = tau[0] - p_a[k];
                if constexpr (has_parent) {
                    tscrew<real> p_prime = p_a + (u[0] / j1dof.H) * j1dof.D;
                    p_a = AdT(Tinv, p_prime);
                }
            } break;
            case JOINT_TYPE_SPHERICAL: {
                u = tau - p_a.w;
                if constexpr (has_parent) {
                    tvec3<real> tau_prime = u;
                    tscrew<real> p_prime = p_a;
                    p_prime.w += j3dof.I_Dinv * tau_prime;
                    p_prime.v += j3dof.Ct_Dinv * tau_prime;
                    p_a = AdT(Tinv, p_prime);
                }
            } break;
        }
    }

    inline void invmass_forward_pass() {
        tscrew<real> a_p = Ad(Tinv, a);
        switch (joint_type) {
            JOINT_DOF_1_CASE {
                int k = get_screw_idx(joint_type);
                udot[0] = (u[0] - dot(j1dof.D, a_p)) / j1dof.H;
                a = a_p;
                a[k] += udot[0];
            } break;
            case JOINT_TYPE_SPHERICAL: {
                udot = j3dof.Dinv * u - j3dof.I_Dinv * a_p.w - glm::transpose(j3dof.Ct_Dinv) * a_p.v;
                a = a_p;
                a.w += udot;
            } break;
        }
    }
};

void featherstone_forward_dynamics(const ArticulatedBodySpec& art,
                                   glm::tvec3<real> gravity, real dt,
                                   const tscrew<real>* f_ext, const real* q, const real* u, const real* tau,
                                   real* udot) {

    int num_joints = art.get_num_joints();
    auto data = new FeatherstoneData[num_joints];

    // Setup
    for (int i = 0; i < num_joints; i++) {
        auto& joint = art.joints[i];
        auto& link = art.links[i];
        uint32_t cur_pos_dof = art.joint_pos_dof_starts[i];
        uint32_t cur_vel_dof = art.joint_vel_dof_starts[i];
        int num_vel_dofs = art.joint_vel_dofs[i];
        data[i].joint_type = joint.type;
        data[i].has_parent = i != 0;
        data[i].Tinv = calc_Tinv(joint, link, q + cur_pos_dof);
        data[i].v0 = calc_v0(joint, u + cur_vel_dof);
        data[i].I_a = tsmat6x6<real>(link.I_j);
        if (f_ext) {
            data[i].f_ext = f_ext[i];
        }
        else {
            data[i].f_ext = tscrew<real>(IDENTITY);
        }
        if (!(i == 0 && art.floating)) {
            for (int j = 0; j < num_vel_dofs; j++) {
                data[i].tau[j] = tau[cur_vel_dof + j];
            }
        }
        data[i].kd = joint.kd;
    }

    // Forward pass 1
    if (art.floating) {
        data[0].v = make_tscrew(u);
        data[0].p_a = -adT(data[0].v, data[0].I_a * data[0].v) - data[0].f_ext - make_tscrew(tau);
    }
    else {
        data[0].v = tscrew<real>(IDENTITY);
        data[0].forward_pass1();
    }
    for (int j = 1; j < num_joints; j++) {
        int i = art.bfs_iteration_order[j];
        data[i].v = data[art.parents[i]].v;
        data[i].forward_pass1();
    }

    // Backward pass
    for (int j = num_joints - 1; j >= 1; j--) {
        int i = art.bfs_iteration_order[j];
        data[i].backward_pass<true>(dt);
        data[art.parents[i]].I_a += data[i].I_a;
        data[art.parents[i]].p_a += data[i].p_a;
    }
    if (!art.floating) {
        data[0].backward_pass<false>(dt);
    }

    // Forward pass 2
    if (art.floating) {
        data[0].a = inverse(data[0].I_a) * (-data[0].p_a);
    }
    else {
        data[0].a = tscrew<real>(tvec3<real>(0), -gravity);
        data[0].forward_pass2();
    }
    for (int j = 1; j < num_joints; j++) {
        int i = art.bfs_iteration_order[j];
        data[i].a = data[art.parents[i]].a;
        data[i].forward_pass2();
    }

    // Output udot
    if (art.floating) {
        ttransform<real> T_root = ttransform(make_vec3(q), glm::mat3_cast(make_quat(q + 3)));
        data[0].a += Ad(inverse(T_root), tscrew<real>(tvec3<real>(0), gravity));
        udot[0] = data[0].a.w[0];
        udot[1] = data[0].a.w[1];
        udot[2] = data[0].a.w[2];
        udot[3] = data[0].a.v[0];
        udot[4] = data[0].a.v[1];
        udot[5] = data[0].a.v[2];
    }
    else {
        int num_vel_dofs = art.joint_vel_dofs[0];
        for (int j = 0; j < num_vel_dofs; j++) {
            udot[j] = data[0].udot[j];
        }
    }
    for (int i = 1; i < num_joints; i++) {
        uint32_t cur_vel_dof = art.joint_vel_dof_starts[i];
        int num_vel_dofs = art.joint_vel_dofs[i];
        for (int j = 0; j < num_vel_dofs; j++) {
            udot[cur_vel_dof + j] = data[i].udot[j];
        }
    }

    delete [] data;
}

// Solves M^{-1} X, using Featherstone's algorithm, where M is the mass matrix.

void multiply_inverse_mass_matrix(const ArticulatedBodySpec& art, real dt,
                                  const real* q, dynmat_view<real> X,
                                  OUT dynmat_view<real> Minv_X) {

    assert(X.rows == art.num_vel_dofs);
    assert(Minv_X.rows == art.num_vel_dofs);
    assert(X.cols == Minv_X.cols);

    int num_joints = art.get_num_joints();
    auto data = new FeatherstoneData[num_joints];

    // Setup
    for (int i = 0; i < num_joints; i++) {
        auto& joint = art.joints[i];
        auto& link = art.links[i];
        uint32_t cur_pos_dof = art.joint_pos_dof_starts[i];
        uint32_t cur_vel_dof = art.joint_vel_dof_starts[i];
        data[i].joint_type = joint.type;
        data[i].has_parent = i != 0;
        data[i].Tinv = calc_Tinv(joint, link, q + cur_pos_dof);
        data[i].I_a = tsmat6x6<real>(link.I_j);
        data[i].kd = joint.kd;
    }

    // Backward pass 1 (same for all items in batch)
    for (int j = num_joints - 1; j >= 1; j--) {
        int i = art.bfs_iteration_order[j];
        data[i].invmass_backward_pass1<true>(dt);
        data[art.parents[i]].I_a += data[i].I_a;
    }
    if (!art.floating) {
        data[0].invmass_backward_pass1<false>(dt);
    }

    for (int b = 0; b < X.cols; b++) {
        // Setup p_a and atu
        if (art.floating) {
            auto tau_root = tscrew<real>(X(0, b), X(1, b), X(2, b), X(3, b), X(4, b), X(5, b));
            data[0].p_a = -tau_root;
        }
        else {
            data[0].p_a = tscrew<real>(IDENTITY);
            int num_vel_dofs = art.joint_vel_dofs[0];
            for (int j = 0; j < num_vel_dofs; j++) {
                data[0].tau[j] = X(j, b);
            }
        }
        for (int i = 1; i < num_joints; i++) {
            data[i].p_a = tscrew<real>(IDENTITY);
            int num_vel_dofs = art.joint_vel_dofs[i];
            uint32_t cur_vel_dof = art.joint_vel_dof_starts[i];
            for (int j = 0; j < num_vel_dofs; j++) {
                data[i].tau[j] = X(cur_vel_dof + j, b);
            }
        }

        // Backward pass 2
        for (int j = num_joints - 1; j >= 1; j--) {
            int i = art.bfs_iteration_order[j];
            data[i].invmass_backward_pass2<true>();
            data[art.parents[i]].p_a += data[i].p_a;
        }
        if (!art.floating) {
            data[0].invmass_backward_pass2<false>();
        }

        // Forward pass
        if (art.floating) {
            data[0].a = inverse(data[0].I_a) * (-data[0].p_a);
        }
        else {
            data[0].a = tscrew<real>(IDENTITY);
            data[0].invmass_forward_pass();
        }
        for (int j = 1; j < num_joints; j++) {
            int i = art.bfs_iteration_order[j];
            data[i].a = data[art.parents[i]].a;
            data[i].invmass_forward_pass();
        }

        // Output Minv_X
        if (art.floating) {
            ttransform<real> T_root = ttransform(make_vec3(q), glm::mat3_cast(make_quat(q + 3)));
            data[0].a += Ad(inverse(T_root), tscrew<real>(IDENTITY));
            Minv_X(0, b) = data[0].a.w[0];
            Minv_X(1, b) = data[0].a.w[1];
            Minv_X(2, b) = data[0].a.w[2];
            Minv_X(3, b) = data[0].a.v[0];
            Minv_X(4, b) = data[0].a.v[1];
            Minv_X(5, b) = data[0].a.v[2];
        }
        else {
            int num_vel_dofs = art.joint_vel_dofs[0];
            for (int j = 0; j < num_vel_dofs; j++) {
                Minv_X(j, b) = data[0].udot[j];
            }
        }
        for (int i = 1; i < num_joints; i++) {
            uint32_t cur_vel_dof = art.joint_vel_dof_starts[i];
            int num_vel_dofs = art.joint_vel_dofs[i];
            for (int j = 0; j < num_vel_dofs; j++) {
                Minv_X(cur_vel_dof + j, b) = data[i].udot[j];
            }
        }
    }

    delete [] data;
}

void mass_matrix_using_rnea(const ArticulatedBodySpec& art, real dt, const real* q, dynmat<real>& M) {
    assert(M.rows == art.num_vel_dofs);
    assert(M.cols == art.num_vel_dofs);

    uint32_t dof = art.get_num_vel_dofs();
    std::vector<real> u(dof, 0);
    std::vector<real> udot(dof, 0);
    std::vector<tscrew<real>> f_ext(art.get_num_joints(), tscrew<real>(IDENTITY));
    std::vector<real> tau(dof, 0);

    udot[0] = 1;
    rne_inverse_dynamics(art, glm::tvec3<real>(0), dt, q, u.data(), udot.data(), f_ext.data(), M.data());
    for (int i = 1; i < dof; i++) {
        udot[i-1] = 0;
        udot[i] = 1;
        rne_inverse_dynamics(art, glm::tvec3<real>(0), dt, q, u.data(), udot.data(), f_ext.data(), M.data() + i*dof);
    }
}

void all_forces(const ArticulatedBodySpec& art, glm::tvec3<real> gravity, real dt, const tscrew<real>* f_ext, const real* q,
                const real* u, real* tau) {
    int dof = art.get_num_vel_dofs();
    std::vector<real> udot(dof, 0);
    rne_inverse_dynamics(art, gravity, dt, q, u, udot.data(), f_ext, tau);
}

void forward_dynamics_using_rnea(const ArticulatedBodySpec& art, glm::tvec3<real> gravity, real dt, const tscrew<real>* f_ext,
                                 const real* q, const real* u, const real* tau, real* udot) {
    using Matrix = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic>;
    using Vector = Eigen::Matrix<real, Eigen::Dynamic, 1>;

    int dof = art.get_num_vel_dofs();
    dynmat<real> M(dof, dof);
    Eigen::Map<Matrix> M_eigen(M.data(), dof, dof);
    Vector h(dof);
    Vector b(dof);
    Vector tau_ext = Eigen::Map<const Vector>(tau, dof);
    mass_matrix_using_rnea(art, dt, q, M);
    all_forces(art, gravity, dt, f_ext, q, u, OUT h.data());
    b.noalias() = tau_ext - h;
    Eigen::Map<Vector> x = Eigen::Map<Vector>(udot, dof);
    x.noalias() = M_eigen.llt().solve(b);
}

void integrate_implicit_euler(const ArticulatedBodySpec& art, real dt, const real* udot, real* q, real* u) {
    if (udot) {
        for (int d = 0; d < art.get_num_vel_dofs(); d++) {
            u[d] += udot[d] * dt;
        }
    }
    real* qi = q; real* qdi = u;
    for (int i = 0; i < art.get_num_joints(); i++) {
        switch (art.joints[i].type) {
            JOINT_DOF_1_CASE {
                qi[0] += qdi[0]*dt;
            } break;
            case JOINT_TYPE_SPHERICAL: {
                qi[0] += 0.5*dt*(qi[3]*qdi[0] + qi[1]*qdi[2] - qi[2]*qdi[1]);
                qi[1] += 0.5*dt*(qi[3]*qdi[1] + qi[2]*qdi[0] - qi[0]*qdi[2]);
                qi[2] += 0.5*dt*(qi[3]*qdi[2] + qi[0]*qdi[1] - qi[1]*qdi[0]);
                qi[3] -= 0.5*dt*(qi[0]*qdi[0] + qi[1]*qdi[1] + qi[2]*qdi[2]);
                real q_len = glm::sqrt(qi[0]*qi[0] + qi[1]*qi[1] + qi[2]*qi[2] + qi[3]*qi[3]);
                qi[0] /= q_len; qi[1] /= q_len; qi[2] /= q_len; qi[3] /= q_len;
            } break;
            case JOINT_TYPE_FLOATING: {
                // TODO: is there a more accurate way to integrate SE(3)?
                glm::tvec3<real> p_dot = make_quat(qi+3) * make_vec3(qdi+3);
                qi[0] += dt*p_dot[0];
                qi[1] += dt*p_dot[1];
                qi[2] += dt*p_dot[2];
                qi[3] += 0.5*dt*(qi[6]*qdi[0] + qi[4]*qdi[2] - qi[5]*qdi[1]);
                qi[4] += 0.5*dt*(qi[6]*qdi[1] + qi[5]*qdi[0] - qi[3]*qdi[2]);
                qi[5] += 0.5*dt*(qi[6]*qdi[2] + qi[3]*qdi[1] - qi[4]*qdi[0]);
                qi[6] -= 0.5*dt*(qi[3]*qdi[0] + qi[4]*qdi[1] + qi[5]*qdi[2]);
                real q_len = glm::sqrt(qi[3]*qi[3] + qi[4]*qi[4] + qi[5]*qi[5] + qi[6]*qi[6]);
                qi[3] /= q_len; qi[4] /= q_len; qi[5] /= q_len; qi[6] /= q_len;
            } break;
        }
        qi += art.joint_pos_dofs[i];
        qdi += art.joint_vel_dofs[i];
    }
}

void calc_transforms(const ArticulatedBodySpec& art, const real* q, glmx::ttransform<real>* T_joint_globals,
                     glmx::ttransform<real>* T_link_globals) {

    for (uint32_t i : art.bfs_iteration_order) {
        int d = art.joint_pos_dof_starts[i];
        auto& joint = art.joints[i];
        auto& link = art.links[i];
        ttransform<real> T_joint_global_parent;
        if (i == 0) {
            T_joint_global_parent = ttransform<real>(IDENTITY);
        }
        else {
            T_joint_global_parent = T_joint_globals[art.parents[i]];
        }
        switch (joint.type) {
            JOINT_DOF_1_CASE {
                tscrew<real> S = get_joint_screw(joint.type);
                T_joint_globals[i] = T_joint_global_parent * ttransform<real>(link.local_joint_pose) * move(S, q[d]);
            } break;
            case JOINT_TYPE_SPHERICAL: {
                glm::tquat<real> q_j = glm::make_quat<real>(q + d);
                T_joint_globals[i] = T_joint_global_parent * ttransform<real>(link.local_joint_pose) * glm::mat3_cast(q_j);
            } break;
            case JOINT_TYPE_FLOATING: {
                glm::tvec3<real> v_j = glm::make_vec3<real>(q + d);
                glm::tquat<real> q_j = glm::make_quat<real>(q + d + 3);
                T_joint_globals[i] = T_joint_global_parent * ttransform<real>(v_j, glm::mat3_cast(q_j));
            } break;
        }
    }
    if (T_link_globals) {
        for (int i = 0; i < art.get_num_joints(); i++) {
            T_link_globals[i] = T_joint_globals[i] * ttransform<real>(art.links[i].local_link_pose);
        }
    }
}

Eigen::Matrix<real, 3, 3> glm_to_eigen(const glm::tmat3x3<real>& M) {
    return Eigen::Map<Eigen::Matrix<real, 3, 3>>((real*)&M[0], 3, 3).transpose();
}

Eigen::Matrix<real, 3, 3> glm_to_eigen(const tsmat3x3<real>& M) {
    Eigen::Matrix<real, 3, 3> Me;
    Me(0, 0) = M.xx; Me(1, 1) = M.yy; Me(2, 2) = M.zz;
    Me(1, 2) = Me(2, 1) = M.yz;
    Me(2, 0) = Me(0, 2) = M.zx;
    Me(0, 1) = Me(1, 0) = M.xy;
    return Me;
}

Eigen::Matrix<real, 6, 6> glm_to_eigen(const tsmat6x6<real>& I) {
    Eigen::Matrix<real, 6, 6> M;
    M.block<3,3>(0, 0) = glm_to_eigen(I.I);
    M.block<3,3>(3, 0) = glm_to_eigen(I.C);
    M.block<3,3>(0, 3) = glm_to_eigen(I.C).transpose();
    M.block<3,3>(3, 3) = glm_to_eigen(I.M);
    return M;
}

void glm_to_dynmat(const tsmat3x3<real>& I, OUT dynmat_view<real> M) {
    M(0, 0) = I.xx;
    M(1, 1) = I.yy;
    M(2, 2) = I.zz;
    M(1, 2) = M(2, 1) = I.yz;
    M(0, 2) = M(2, 0) = I.zx;
    M(0, 1) = M(1, 0) = I.xy;
}

void glm_to_dynmat(const tmat3x3<real>& C, OUT dynmat_view<real> M) {
    M(0, 0) = C[0][0];
    M(0, 1) = C[0][1];
    M(0, 2) = C[0][2];
    M(1, 0) = C[1][0];
    M(1, 1) = C[1][1];
    M(1, 2) = C[1][2];
    M(2, 0) = C[2][0];
    M(2, 1) = C[2][1];
    M(2, 2) = C[2][2];
}

void glm_to_dynmat(const tsmat6x6<real>& I, OUT dynmat_view<real> M) {
    glm_to_dynmat(I.I, M.slice(0, 3, 0, 3));
    glm_to_dynmat(I.C, M.slice(0, 3, 3, 3));
    glm_to_dynmat(glm::transpose(I.C), M.slice(3, 3, 0, 3));
    glm_to_dynmat(I.M, M.slice(3, 3, 3, 3));
}

void mass_matrix(const ArticulatedBodySpec& art, real dt, const real* q, dynmat_view<real> M) {
    assert(M.rows == art.num_vel_dofs);
    assert(M.cols == art.num_vel_dofs);

    using namespace Eigen;
    uint32_t vdof = art.get_num_vel_dofs();
    uint32_t num_joints = art.get_num_joints();
    M.clear_zero();

    std::vector<tsmat6x6<real>> I(num_joints);

    // Calculates Fi to M.
#define CRBA_COPY_Fi_TO_M(k) \
        M(0, vpos_i + k) = M(vpos_i + k, 0) = Fi[k][0]; \
        M(1, vpos_i + k) = M(vpos_i + k, 1) = Fi[k][1]; \
        M(2, vpos_i + k) = M(vpos_i + k, 2) = Fi[k][2]; \
        M(3, vpos_i + k) = M(vpos_i + k, 3) = Fi[k][3]; \
        M(4, vpos_i + k) = M(vpos_i + k, 4) = Fi[k][4]; \
        M(5, vpos_i + k) = M(vpos_i + k, 5) = Fi[k][5];

    std::vector<ttransform<real>> Tinv(num_joints);
    std::vector<ttransform<real>> T_flink(num_joints); // used when art.floating == true

    for (uint32_t i : art.bfs_iteration_order) {
        uint32_t ppos = art.joint_pos_dof_starts[i];
        uint32_t vpos = art.joint_vel_dof_starts[i];
        Tinv[i] = calc_Tinv(art.joints[i], art.links[i], q + ppos);
        auto I0 = tspmat<real>(tsmat3x3<real>(art.links[i].inertia), glm::tvec3<real>(0), art.links[i].mass);
        I[i] = tsmat6x6<real>(art.links[i].I_j);

        if (art.floating) {
            if (i == 0) T_flink[i] = ttransform<real>(IDENTITY);
            else T_flink[i] = T_flink[art.parents[i]] * Tinv[i];
        }
    }

    int l_finish = art.floating? 1 : 0;
    for (int l = num_joints-1; l >= l_finish; l--) {
        uint32_t i = art.bfs_iteration_order[l];
        uint32_t vpos_i = art.joint_vel_dof_starts[i];
        uint32_t vdof_i = art.joint_vel_dofs[i];

        if (i != 0) {
            I[art.parents[i]] += inv_transform(I[i], Tinv[i]);
        }
        real kd = art.joints[i].kd;
        switch (art.joints[i].type) {
            JOINT_DOF_1_CASE {
                int ti = get_screw_idx(art.joints[i].type);
                tscrew<real> Fi[1] = { I[i][ti] };
                // CRBA_Ft_S(i, i, 0, 0);
                M(vpos_i, vpos_i) = Fi[0][ti] + kd*dt;
                uint32_t j = i;
                while (j != 0) {
                    Fi[0] = AdT(Tinv[j], Fi[0]);
                    j = art.parents[j];
                    uint32_t vpos_j = art.joint_vel_dof_starts[j];
                    uint32_t vdof_j = art.joint_vel_dofs[j];
                    if (vdof_j == 1) {
                        int tj = get_screw_idx(art.joints[j].type);
                        M(vpos_i, vpos_j) = M(vpos_j, vpos_i) = Fi[0][tj];
                    }
                    else if (vdof_j == 3) {
                        M(vpos_i, vpos_j+0) = M(vpos_j+0, vpos_i) = Fi[0][0];
                        M(vpos_i, vpos_j+1) = M(vpos_j+1, vpos_i) = Fi[0][1];
                        M(vpos_i, vpos_j+2) = M(vpos_j+2, vpos_i) = Fi[0][2];
                    }
                }
                if (art.floating) {
                    Fi[0] = AdT(T_flink[j], Fi[0]);
                    CRBA_COPY_Fi_TO_M(0);
                }
            } break;
            case JOINT_TYPE_SPHERICAL: {
                tscrew<real> Fi[3] = {I[i][0], I[i][1], I[i][2]};
                M(vpos_i+0, vpos_i+0) = Fi[0][0] + kd*dt;
                M(vpos_i+0, vpos_i+1) = M(vpos_i+1, vpos_i+0) = Fi[0][1];
                M(vpos_i+0, vpos_i+2) = M(vpos_i+2, vpos_i+0) = Fi[0][2];
                M(vpos_i+1, vpos_i+1) = Fi[1][1] + kd*dt;
                M(vpos_i+1, vpos_i+2) = M(vpos_i+2, vpos_i+1) = Fi[1][2];
                M(vpos_i+2, vpos_i+2) = Fi[2][2] + kd*dt;
                uint32_t j = i;
                while (j != 0) {
                    Fi[0] = AdT(Tinv[j], Fi[0]);
                    Fi[1] = AdT(Tinv[j], Fi[1]);
                    Fi[2] = AdT(Tinv[j], Fi[2]);
                    j = art.parents[j];
                    uint32_t vpos_j = art.joint_vel_dof_starts[j];
                    uint32_t vdof_j = art.joint_vel_dofs[j];
                    if (vdof_j == 1) {
                        int tj = get_screw_idx(art.joints[j].type);
                        M(vpos_i+0, vpos_j+0) = M(vpos_j+0, vpos_i+0) = Fi[0][tj];
                        M(vpos_i+1, vpos_j+0) = M(vpos_j+0, vpos_i+1) = Fi[1][tj];
                        M(vpos_i+2, vpos_j+0) = M(vpos_j+0, vpos_i+2) = Fi[2][tj];
                    }
                    else if (vdof_j == 3) {
                        M(vpos_i+0, vpos_j+0) = M(vpos_j+0, vpos_i+0) = Fi[0][0];
                        M(vpos_i+0, vpos_j+1) = M(vpos_j+1, vpos_i+0) = Fi[0][1];
                        M(vpos_i+0, vpos_j+2) = M(vpos_j+2, vpos_i+0) = Fi[0][2];
                        M(vpos_i+1, vpos_j+0) = M(vpos_j+0, vpos_i+1) = Fi[1][0];
                        M(vpos_i+1, vpos_j+1) = M(vpos_j+1, vpos_i+1) = Fi[1][1];
                        M(vpos_i+1, vpos_j+2) = M(vpos_j+2, vpos_i+1) = Fi[1][2];
                        M(vpos_i+2, vpos_j+0) = M(vpos_j+0, vpos_i+2) = Fi[2][0];
                        M(vpos_i+2, vpos_j+1) = M(vpos_j+1, vpos_i+2) = Fi[2][1];
                        M(vpos_i+2, vpos_j+2) = M(vpos_j+2, vpos_i+2) = Fi[2][2];
                    }
                }
                if (art.floating) {
                    for (int k = 0; k < 3; k++) {
                        Fi[k] = AdT(T_flink[j], Fi[k]);
                        CRBA_COPY_Fi_TO_M(k);
                    }
                }
            } break;
            default: break;
        }
    }
    if (art.floating) {
        glm_to_dynmat(I[0], M.slice(0, 6, 0, 6));
    }
}

void calc_reduced_to_maximal_jacobian(
        const ArticulatedBodySpec& art,
        const tscrew<real>* J,
        OUT dynmat_view<real> J_mr, OUT dynmat_view<real> J_mr_dot) {
}

}