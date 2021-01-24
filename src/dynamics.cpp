//
// Created by Phillip Chang on 2020/09/12.
//

#include "artsim/dynamics.h"

#define SHOW_LOG

#ifdef SHOW_LOG
#define output_log(...) printf(__VA_ARGS__)
#else
#define output_log(...)
#endif

namespace artsim {

void calc_S(const ArticulatedBody &art, const real *q, tscrew<real> *S) {
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
        case JOINT_TYPE_FLOATING: return ttransform<real>();
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
        default: return ttransform<real>();
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
        default: return tscrew<real>();
    }
}

void
calculate_jacobian_for_local_frame(const ArticulatedBody& art, uint32_t link_idx, const ttransform<real>& T_contact,
                                   const artsim::ttransform<real>* T_joint_global, const tscrew<real>* S,
                                   tscrew<real>* J_local) {
    int num_vel_dofs = art.get_num_vel_dofs();
    int num_joints = art.get_num_joints();

    std::fill_n(J_local, num_vel_dofs, tscrew<real>());

    int i = link_idx;

    do {
        uint32_t vel_starts = art.joint_vel_dof_starts[i];
        uint32_t vel_dof = art.joint_vel_dofs[i];

        auto T_rel = T_joint_global[i] / T_contact;
        for (int j = vel_starts; j < vel_starts + vel_dof; j++) {
            J_local[j] = Ad(T_rel, S[j]);
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
    real dt;

    // INTERMEDIATE VALUES
    ttransform<real> T_global_inv;

    // OUT
    tscrew<real> v;
    tscrew<real> a;
    tscrew<real> f;
    tvec3<real> tau;           // dof

    // kin must be calculated using jcalc() before this call
    void rnea_pass1() {
        if (has_parent) T_global_inv = T_global_inv * Tinv;
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
        f = I * a - adT(v, I * v) - AdT(T_global_inv, f_ext);
    }

    void rnea_pass2() {
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

void rne_inverse_dynamics(const ArticulatedBody& art, glm::tvec3<real> gravity, real dt,
                          const real* q, const real* u, const real* udot, const tscrew<real>* f_ext,
                          real* tau) {

    int num_joints = art.get_num_joints();
    std::vector<RecursiveNewtonEulerData> data(num_joints);

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
        auto I0 = tspmat<real>(tsmat3x3<real>(art.links[i].inertia), glm::tvec3<real>(0), art.links[i].mass);
        data[i].I = inv_transform(I0, ttransform<real>(inverse(art.links[i].local_link_pose)));
        if (f_ext) data[i].f_ext = f_ext[i];
        if (!art.floating || i != 0) {
            for (int j = 0; j < num_vel_dofs; j++) {
                data[i].udot[j] = udot[cur_vel_dof + j];
            }
        }
        data[i].kd = joint.kd;
        data[i].dt = dt;
    }

    for (int i : art.bfs_iteration_order) {
        if (i == 0) {
            if (art.floating) {
                ttransform<real> T_root = ttransform<real>(make_vec3(q), glm::mat3_cast(make_quat(q + 3)));
                data[0].T_global_inv = ttransform<real>();
                data[0].v = make_tscrew(u);
                data[0].a = Ad(inverse(T_root), tscrew<real>(tvec3<real>(0), -gravity));
                data[0].f = data[0].I * data[0].a - adT(data[0].v, data[0].I * data[0].v) - data[0].f_ext;
                continue;
            }
            else {
                data[0].v = tscrew<real>();
                data[0].a = tscrew<real>(tvec3<real>(0), -gravity);
                data[0].T_global_inv = ttransform<real>();
            }
        }
        else {
            data[i].T_global_inv = data[art.parents[i]].T_global_inv;
            data[i].v = data[art.parents[i]].v;
            data[i].a = data[art.parents[i]].a;
        }
        data[i].rnea_pass1();
    }

    int j_limit = art.floating? 1 : 0;
    for (int j = num_joints - 1; j >= j_limit; j--) {
        int i = art.bfs_iteration_order[j];
        data[i].rnea_pass2();
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
    real dt;

    // INTERMEDIATE VALUES
    ttransform<real> T_global_inv;
    tsmat6x6<real> I_a;
    tscrew<real> p_a;
    tscrew<real> v;
    tscrew<real> c;
    struct {
        tscrew<real> D;
        real H;
    } j1dof;
    struct {
        tsmat3x3<real> Dinv;
        tsmat3x3<real> I_Dinv;
        tmat3x3<real> Ct_Dinv;
    } j3dof;
    tvec3<real> u; // dof
    tscrew<real> a;

    // OUT
    tvec3<real> udot;         // dof

    // kin must be calculated using jcalc() before this call
    inline void featherstone_pass1() {
        if (has_parent) T_global_inv = T_global_inv * Tinv;
        v = Ad(Tinv, v) + v0;
        c = ad(v, v0); // + c0; (c0 is zero for all joints)
        p_a = -adT(v, I_a * v) - AdT(T_global_inv, f_ext);
    }

    inline void featherstone_pass2() {
        switch (joint_type) {
            JOINT_DOF_1_CASE {
                int k = get_screw_idx(joint_type);
                j1dof.D = I_a[k];
                j1dof.H = j1dof.D[k] + kd * dt;
                u[0] = tau[0] - p_a[k];
                if (has_parent) {
                    tsmat6x6<real> I_prime = I_a - symmetric_cartesian_product(j1dof.D) / j1dof.H;
                    tscrew<real> p_prime = p_a + I_prime * c + ((tau[0] - kd * v0[k] - p_a[k]) / j1dof.H) * j1dof.D;
                    I_a = inv_transform(I_prime, Tinv);
                    p_a = AdT(Tinv, p_prime);
                }
            } break;
            case JOINT_TYPE_SPHERICAL: {
                j3dof.Dinv = inverse(I_a.I + tsmat3x3<real>(kd * dt));
                j3dof.I_Dinv = I_a.I * j3dof.Dinv;
                j3dof.Ct_Dinv = glm::transpose(I_a.C) * mat3_cast(j3dof.Dinv);
                u = tau - p_a.w;
                if (has_parent) {
                    auto I_prime = tsmat6x6<real>(
                            I_a.I - j3dof.I_Dinv * I_a.I,
                            I_a.C - mat3_cast(j3dof.I_Dinv) * I_a.C,
                            I_a.M - smat3_cast(j3dof.Ct_Dinv * I_a.C));
                    tvec3<real> tau_prime = tau - kd * v0.w - p_a.w;
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

    inline void featherstone_pass3() {
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
};

void featherstone_forward_dynamics(const ArticulatedBody& art, glm::tvec3<real> gravity, real dt,
                                   const tscrew<real>* f_ext, const real* q, const real* u, const real* tau,
                                   real* udot) {

    int num_joints = art.get_num_joints();
    std::vector<FeatherstoneData> data(num_joints);

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
        auto I0 = tspmat<real>(tsmat3x3<real>(link.inertia), glm::tvec3<real>(0), link.mass);
        data[i].I_a = tsmat6x6<real>(inv_transform(I0, ttransform<real>(inverse(link.local_link_pose))));
        if (f_ext) data[i].f_ext = f_ext[i];
        if (!(i == 0 && art.floating)) {
            for (int j = 0; j < num_vel_dofs; j++) {
                data[i].tau[j] = tau[cur_vel_dof + j];
            }
        }
        data[i].kd = joint.kd;
        data[i].dt = dt;
    }

    for (int i : art.bfs_iteration_order) {
        if (i == 0) {
            if (art.floating) {
                data[0].T_global_inv = ttransform<real>();
                data[0].v = make_tscrew(u);
                data[0].p_a = -adT(data[0].v, data[0].I_a * data[0].v) - data[0].f_ext - make_tscrew(tau);
                continue;
            }
            else {
                data[i].T_global_inv = ttransform<real>();
                data[i].v = tscrew<real>();
            }
        }
        else {
            data[i].T_global_inv = data[art.parents[i]].T_global_inv;
            data[i].v = data[art.parents[i]].v;
        }
        data[i].featherstone_pass1();
    }
    int j_limit = art.floating? 1 : 0;
    for (int j = num_joints - 1; j >= j_limit; j--) {
        int i = art.bfs_iteration_order[j];
        data[i].featherstone_pass2();
        if (i != 0) {
            data[art.parents[i]].I_a += data[i].I_a;
            data[art.parents[i]].p_a += data[i].p_a;
        }
    }
    for (int i : art.bfs_iteration_order) {
        if (i == 0) {
            if (art.floating) {
                data[0].a = inverse(data[0].I_a) * (-data[0].p_a);
                continue;
            }
            else {
                data[i].a = tscrew<real>(tvec3<real>(0), -gravity);
            }
        }
        else {
            data[i].a = data[art.parents[i]].a;
        }
        data[i].featherstone_pass3();
    }
    for (int i = 0; i < num_joints; i++) {
        uint32_t cur_vel_dof = art.joint_vel_dof_starts[i];
        int num_vel_dofs = art.joint_vel_dofs[i];
        if (i == 0 && art.floating) {
            ttransform<real> T_root = ttransform(make_vec3(q), glm::mat3_cast(make_quat(q + 3)));
            data[i].a += Ad(inverse(T_root), tscrew<real>(tvec3<real>(0), gravity));
            udot[0] = data[0].a.w[0];
            udot[1] = data[0].a.w[1];
            udot[2] = data[0].a.w[2];
            udot[3] = data[0].a.v[0];
            udot[4] = data[0].a.v[1];
            udot[5] = data[0].a.v[2];
        }
        else {
            for (int j = 0; j < num_vel_dofs; j++) {
                udot[cur_vel_dof + j] = data[i].udot[j];
            }
        }
    }
}

inline real compute_r(real theta, glm::tvec3<real> Minv_r3, real c_z, real mu) {
    return -c_z / (Minv_r3.z / mu + Minv_r3.x * cos(theta) + Minv_r3.y * sin(theta));
}

inline real bisection_gradient(const tsmat3x3<real>& Minv, tvec3<real> c, tvec3<real> lambda, real mu) {
    tsmat3x3<real> M = inverse(Minv);
    glm::tvec3<real> M_r3 = tvec3<real>(M.zx, M.yz, M.zz);
    glm::tvec3<real> eta = glm::cross(M_r3, glm::tvec3<real>(lambda.x, lambda.y, -mu*mu*lambda.z));
    return glm::dot(Minv * lambda + c, eta);
}

static glm::tvec3<real> contact_bisection_solver(tvec3<real> lambda_v0, const tsmat3x3<real>& Minv, tvec3<real> c, real mu) {
    const real gamma = 1e-4;
    real theta = glm::atan(lambda_v0.y, lambda_v0.x);
    tvec3<real> Minv_r3 = tvec3<real>(Minv.zx, Minv.yz, Minv.zz);
    real r = compute_r(theta, Minv_r3, c.z, mu);
    real lambda_z = r / mu;
    tvec3<real> lambda = tvec3<real>(r*cos(theta), r*sin(theta), lambda_z);
    real D0 = bisection_gradient(Minv, c, lambda, mu);

    real theta_p_delta = glm::asin(mu*lambda_v0.z / glm::sqrt(lambda_v0.x*lambda_v0.x + lambda_v0.y*lambda_v0.y));
    real theta_p;
    if (D0 >= 0) {
        theta_p = theta - pi<real>()/2 + theta_p_delta;
    }
    else {
        theta_p = theta + pi<real>()/2 - theta_p_delta;
    }

    int iter = 0;
    tvec3<real> lambda_b;
    real theta_orig = theta;
    real theta_p_orig = theta_p;
    do {
        real theta_b = 0.5 * (theta + theta_p);
        real r_b = compute_r(theta_b, Minv_r3, c.z, mu);
        real lambda_z_b = r_b / mu;
        lambda_b = vec3(r_b*cos(theta_b), r_b*sin(theta_b), lambda_z_b);
        real grad = bisection_gradient(Minv, c, lambda_b, mu);
        if (grad * D0 > 0) { theta_p = theta_b; }
        else { theta = theta_b; }
        iter++;
        if (iter == 100) {
            output_log("Bisection solver: bisection cannot find root!\n");
            exit(EXIT_FAILURE);
        }
    }
    while (abs(theta - theta_p) >= gamma);
    // output_log("Bisection solver: bisection finished in %d iters\n", iter);

    return lambda_b;
}

static tvec3<real> contact_projection_solver(tvec3<real> lambda, const tsmat3x3<real>& Minv, tvec3<real> c, real mu) {
    const real alpha = 1.0f;
    real r_z = alpha / Minv.zz;
    real r_t = alpha / max(Minv.xx, Minv.yy);
    tvec3<real> v = c + Minv*lambda;
    real lambda_z = max(real(0), lambda.z - r_z*v.z);
    tvec2<real> lambda_t = tvec2<real>(lambda.x - r_t*v.x, lambda.y - r_t*v.y);
    real lambda_t_len = length(lambda_t);
    if (lambda_t_len > mu*lambda_z) {
        lambda_t = mu*lambda_z*normalize(lambda_t);
    }
    return tvec3<real>(lambda_t.x, lambda_t.y, lambda_z);
}

static std::tuple<glm::tvec3<real>, real, bool> contact_ncp_solver(tvec3<real> lambda_v0,
                                                                const tsmat3x3<real>& Minv, tvec3<real> c, real mu, real r) {
    // Initial value for lambda
    tvec3<real> lambda = lambda_v0;
    tvec3<real> lambda_prev = lambda;

    // Damping parameter for Newton method
    const real alpha = real(0.75);

    // Newton-Raphson method with NCP formulation
    real ncp_error_sq;
    real ncp_error_sq_prev = 1e8;
    real w;

    bool success = true;

    for (int i = 0; i < 16; i++) {
        tvec3<real> v = c + Minv*lambda;

        // Calculate Jacobian of the current system
        tsmat3x3<real> J;
        {
            real a = glm::sqrt(v.x*v.x + v.y*v.y);
            real b = r*(mu*lambda.z - glm::sqrt(lambda.x*lambda.x + lambda.y*lambda.y));
            real d = glm::sqrt(a*a + b*b);
            w = (d - b) / (a + r*mu*lambda.z - d);

            J.xx = Minv.xx + w;
            J.xy = Minv.xy;
            J.yy = Minv.yy + w;
        }
        {
            real d = glm::sqrt(v.z*v.z + r*r*lambda.z*lambda.z);
            real dphi_dvn = real(1) - v.z/d;

            J.zx = dphi_dvn * Minv.zx;
            J.yz = dphi_dvn * Minv.yz;
            J.zz = r*(real(1) - r*lambda.z/d) + dphi_dvn * Minv.zz;
        }

        // NCP functions
        tvec3<real> phi;
        phi.x = v.x + w * lambda.x;
        phi.y = v.y + w * lambda.y;
        phi.z = glm::sqrt(v.z*v.z + lambda.z*lambda.z) - v.z - lambda.z;

        // Newton step
        lambda -= alpha * (inverse(J) * phi);

        phi.x = v.x + w * lambda.x;
        phi.y = v.y + w * lambda.y;
        phi.z = glm::sqrt(v.z*v.z + lambda.z*lambda.z) - v.z - lambda.z;

        ncp_error_sq = glm::length2(phi);
        output_log("NCP error: %f\n", ncp_error_sq);
        if (ncp_error_sq > ncp_error_sq_prev) {
            // Newton method failed
            lambda = lambda_prev;
            ncp_error_sq = ncp_error_sq_prev;
            success = false;
            break;
        }

        ncp_error_sq_prev = ncp_error_sq;
        lambda_prev = lambda;

        if (ncp_error_sq <= real(1e-8)) {
            // Newton method finished
            break;
        }
    }

    return {lambda, ncp_error_sq, success};
}

/*
 * TODO for contact solver:
 * - PGS:
 *      - Use nesterov momentum for better convergence (See https://apps.dtic.mil/dtic/tr/fulltext/u2/1081106.pdf)
 * - Bisection:
 *      - Still seems to be unstable. Investigate why.
 */

void solve_collision(ContactSolverType type,
                     uint32_t max_iters,
                     const ArticulatedBody& art, const MaterialDB& material_db, glm::tvec3<real> gravity, real dt,
                     const real* q, const real* u, const real* udot_orig, const tscrew<real>* f_ext, const real* tau,
                     const ContactPoint* contact_points, uint32_t num_contact_points, glm::tvec3<real>* out_lambda,
                     real* out_contact_forces) {

    using namespace Eigen;

    int num_vel_dofs = art.get_num_vel_dofs();
    int num_joints = art.get_num_joints();

    Eigen::Matrix<real, Dynamic, 1> u_bar(num_vel_dofs);
    for (int i = 0; i < num_vel_dofs; i++) {
        u_bar[i] = u[i] + udot_orig[i] * dt;
    }

    Eigen::Matrix<real, Dynamic, Dynamic, RowMajor> Jc(3 * num_contact_points, num_vel_dofs);
    Jc.setZero();

    std::vector<tscrew<real>> S(num_vel_dofs);
    calc_S(art, q, OUT S.data());

    std::vector<ttransform<real>> T_link_global(num_joints), T_joint_global(num_joints);
    calc_transforms(art, q, OUT T_link_global.data(), OUT T_joint_global.data());

    std::vector<tscrew<real>> J_local(num_vel_dofs);
    for (int c = 0; c < num_contact_points; c++) {
        const auto& contact_point = contact_points[c];
        if (contact_point.body1_id.is_articulation() && contact_point.body2_id.index == 0) {
            auto [art_id, art_link_idx] = contact_point.body1_id.get_articulation_id();
            calculate_jacobian_for_local_frame(art, art_link_idx,
                                               ttransform<real>(contact_point.T_global), T_joint_global.data(), S.data(),
                                               OUT J_local.data());
            for (int i = 0; i < num_vel_dofs; i++) {
                Jc(3*c + 0, i) = J_local[i].v[0];
                Jc(3*c + 1, i) = J_local[i].v[1];
                Jc(3*c + 2, i) = J_local[i].v[2];
            }
        }
    }

    Eigen::Matrix<real, Dynamic, 1> tau_star = Jc * u_bar;

    dynmat<tsmat3x3<real>> M_contact_inv(num_contact_points, num_contact_points);

    Eigen::Matrix<real, Dynamic, Dynamic> Minv_Jc_T(num_vel_dofs, 3*num_contact_points);
    std::vector<real> zero_vec(num_vel_dofs, 0);
    for (int c = 0; c < 3*num_contact_points; c++) {
        featherstone_forward_dynamics(art, glm::tvec3<real>(0), dt, f_ext, q, zero_vec.data(), Jc.data() + c*num_vel_dofs,
                                      OUT Minv_Jc_T.data() + c*num_vel_dofs);
    }

    for (int k = 0; k < num_contact_points; k++) {
        Eigen::Matrix<real, Dynamic, 3> Minv_Jck_T = Minv_Jc_T.middleCols(3*k, 3);
        for (int i = 0; i < num_contact_points; i++) {
            Eigen::Matrix<real, 3, Dynamic> Jci = Jc.middleRows(3*i, 3);
            Eigen::Matrix<real, 3, 3> M_contact_inv_eigen = Jci * Minv_Jck_T;
            M_contact_inv(i, k) = tsmat3x3<real>(
                    M_contact_inv_eigen(0, 0),
                    M_contact_inv_eigen(1, 1),
                    M_contact_inv_eigen(2, 2),
                    M_contact_inv_eigen(1, 2),
                    M_contact_inv_eigen(2, 0),
                    M_contact_inv_eigen(0, 1));
        }
    }

    std::vector<tvec3<real>> c(num_contact_points);
    std::vector<tvec3<real>> lambda(num_contact_points, tvec3<real>(0));

    const real beta = 0.1;
    const real slop = 1e-4;

    const real mu = 1.0;

    real alpha_min, gamma, lambda_sq_tol, ncp_error_sq_tol;
    real alpha, total_ncp_error_sq;

    switch (type) {
        case ContactSolverType::PGS:
            alpha = 0.75;
            alpha_min = 0.75;
            gamma = 1.0;
            lambda_sq_tol = 1e-6;
            break;
        case ContactSolverType::Bisection:
            alpha = 1.0;
            alpha_min = 0.7;
            gamma = 0.99;
            lambda_sq_tol = 1e-6;
            break;
        case ContactSolverType::NCP:
            alpha = 1.0;
            alpha_min = 1.0;
            gamma = 1.0;
            lambda_sq_tol = 1e-6;
            ncp_error_sq_tol = 1e-6;
            total_ncp_error_sq = 0;
            break;
    }

    for (int i = 0; i < num_contact_points; i++) {
        c[i] = make_vec3<real>(tau_star.data() + 3*i);
        c[i].z -= beta/dt*glm::max<real>(contact_points[i].depth - slop, 0);
    }

    real lambda_norm2;
    std::vector<tvec3<real>> lambda_old(num_contact_points);
    int iter;
    for (iter = 0; iter < max_iters; iter++) {
        std::copy(lambda.begin(), lambda.end(), lambda_old.begin());

        for (int i = 0; i < num_contact_points; i++) {
            const auto& contact_point = contact_points[i];
            if (c[i].z > 0) {
                lambda[i] = (1 - alpha)*lambda[i];
            }
            else {
                tsmat3x3<real> M_inv_ii = M_contact_inv(i, i);
                tvec3<real> lambda_v0 = -(inverse(M_inv_ii) * c[i]);
                if (mu*mu * lambda_v0.z*lambda_v0.z >= lambda_v0.x*lambda_v0.x + lambda_v0.y*lambda_v0.y) {
                    lambda[i] = alpha * lambda_v0 + (1 - alpha) * lambda[i];
                }
                else {
                    tvec3<real> lambda_star;
                    switch (type) {
                        case ContactSolverType::PGS: {
                            lambda_star = contact_projection_solver(lambda[i], M_inv_ii, c[i], mu);
                        } break;
                        case ContactSolverType::Bisection: {
                            lambda_star = contact_bisection_solver(lambda_v0, M_inv_ii, c[i], mu);
                        } break;
                        case ContactSolverType::NCP: {
                            real ncp_error_sq, success;
                            std::tie(lambda_star, ncp_error_sq, success) = contact_ncp_solver(lambda_v0, M_inv_ii, c[i], mu, dt);
                        } break;
                    }
                    lambda[i] = alpha * lambda_star + (1 - alpha) * lambda[i];
                }
            }

            for (int ip = 0; ip < num_contact_points; ip++) {
                if (i == ip) continue;
                tsmat3x3<real> M_ip_i_inv = M_contact_inv(ip, i);
                c[ip] += M_ip_i_inv*(lambda[i] - lambda_old[i]);
            }
        }
        alpha = alpha_min + gamma * (alpha - alpha_min);

        lambda_norm2 = 0.0;
        for (int i = 0; i < num_contact_points; i++) {
            lambda_norm2 += length2(lambda[i] - lambda_old[i]);
        }
        if (lambda_norm2 < lambda_sq_tol) {
            iter++; break;
        }
    }
    if (iter == max_iters) {
        output_log("Contact solver did not converge! (error = %f)\n", sqrt(lambda_norm2));
    }
    else {
        output_log("Contact solver converged in %d iters\n", iter);
    }

    Eigen::Matrix<real, Dynamic, 1> lambda_vec = Map<Eigen::Matrix<real, Dynamic, 1>>((real*)lambda.data(), 3*num_contact_points);
    Eigen::Matrix<real, Dynamic, 1> contact_forces = Jc.transpose() * lambda_vec / dt;

    if (out_lambda) {
        std::memcpy(out_lambda, lambda.data(), sizeof(tvec3<real>) * num_contact_points);
    }
    if (out_contact_forces) {
        std::memcpy(out_contact_forces, contact_forces.data(), sizeof(real) * num_vel_dofs);
    }
}

void mass_matrix_using_rnea(const ArticulatedBody& art, real dt, const real* q, real* M) {
    uint32_t dof = art.get_num_vel_dofs();
    std::vector<real> u(dof, 0);
    std::vector<real> udot(dof, 0);
    std::vector<tscrew<real>> f_ext(art.get_num_joints(), tscrew<real>());
    std::vector<real> tau(dof, 0);

    udot[0] = 1;
    rne_inverse_dynamics(art, glm::tvec3<real>(0), dt, q, u.data(), udot.data(), f_ext.data(), OUT M);
    for (int i = 1; i < dof; i++) {
        udot[i-1] = 0;
        udot[i] = 1;
        rne_inverse_dynamics(art, glm::tvec3<real>(0), dt, q, u.data(), udot.data(), f_ext.data(), OUT M + i*dof);
    }
}

void all_forces(const ArticulatedBody& art, glm::tvec3<real> gravity, real dt, const tscrew<real>* f_ext, const real* q,
                const real* u, real* tau) {
    int dof = art.get_num_vel_dofs();
    std::vector<real> udot(dof, 0);
    rne_inverse_dynamics(art, gravity, dt, q, u, udot.data(), f_ext, tau);
}

void forward_dynamics_using_rnea(const ArticulatedBody& art, glm::tvec3<real> gravity, real dt, const tscrew<real>* f_ext,
                                 const real* q, const real* u, const real* tau, real* udot) {
    using Matrix = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic>;
    using Vector = Eigen::Matrix<real, Eigen::Dynamic, 1>;

    int dof = art.get_num_vel_dofs();
    Matrix M(dof, dof);
    Vector h(dof);
    Vector b(dof);
    Vector tau_ext = Eigen::Map<const Vector>(tau, dof);
    mass_matrix_using_rnea(art, dt, q, M.data());
    all_forces(art, gravity, dt, f_ext, q, u, OUT h.data());
    b.noalias() = tau_ext - h;
    Eigen::Map<Vector> x = Eigen::Map<Vector>(udot, dof);
    x.noalias() = M.llt().solve(b);
}

void integrate_implicit_euler(const ArticulatedBody& art, real dt, const real* udot, real* q, real* u) {
    for (int d = 0; d < art.get_num_vel_dofs(); d++) {
        u[d] += udot[d] * dt;
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

void calc_transforms(const ArticulatedBody& art, const real* q, ttransform<real>* T_link_globals,
                     ttransform<real>* T_joint_globals) {

    for (uint32_t i : art.bfs_iteration_order) {
        int d = art.joint_pos_dof_starts[i];
        auto& joint = art.joints[i];
        auto& link = art.links[i];
        ttransform<real> T_joint_global_parent;
        if (i == 0) {
            T_joint_global_parent = ttransform<real>();
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
        T_link_globals[i] = T_joint_globals[i] * ttransform<real>(link.local_link_pose);
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

void mass_matrix(const ArticulatedBody& art, real dt, const real* q, real* M_ptr) {
    using namespace Eigen;
    uint32_t vdof = art.get_num_vel_dofs();
    uint32_t num_joints = art.get_num_joints();
    Eigen::Map<Eigen::Matrix<real, Dynamic, Dynamic>> M(M_ptr, vdof, vdof);
    M.setZero();

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
        I[i] = tsmat6x6<real>(inv_transform(I0, ttransform<real>(inverse(art.links[i].local_link_pose))));

        if (art.floating) {
            if (i == 0) T_flink[i] = ttransform<real>();
            else T_flink[i] = Tinv[i] * T_flink[art.parents[i]];
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
                M(vpos_i, vpos_i) = M(vpos_i, vpos_i) = Fi[0][ti] + kd*dt;
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
                    Fi[0] = AdT(T_flink[i], Fi[0]);
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
                        Fi[k] = AdT(T_flink[i], Fi[k]);
                        CRBA_COPY_Fi_TO_M(k);
                    }
                }
            } break;
            default: break;
        }
    }
    if (art.floating) {
        M.block<6,6>(0, 0) = glm_to_eigen(I[0]);
    }
}

void
euler_step_with_collision(ContactSolverType type, uint32_t max_iters,
                          const ArticulatedBody& art, const MaterialDB& material_db, glm::tvec3<real> gravity, real dt,
                          const tscrew<real>* f_ext, const real* tau, const ContactPoint* contact_points,
                          uint32_t num_contact_points, real* q, real* u, real* udot, glm::tvec3<real>* lambda) {
    int num_vel_dofs = art.get_num_vel_dofs();
    int num_joints = art.get_num_joints();

    std::vector<real> udot_bar(num_vel_dofs);
    featherstone_forward_dynamics(art, gravity, dt, f_ext, q, u, tau, OUT udot_bar.data());
    // forward_dynamics_using_rnea(art, gravity, dt, f_ext, q, u, tau, OUT udot_bar.data());

    if (num_contact_points == 0) {
        integrate_implicit_euler(art, dt, udot_bar.data(), INOUT q, INOUT u);
    }
    else {
        auto t1 = std::chrono::high_resolution_clock::now();
        std::vector<real> tau_contact(num_vel_dofs);
        std::vector<real> tau_total(num_vel_dofs);

        solve_collision(type, max_iters,
                        art, material_db, gravity, dt,
                        q, u, udot_bar.data(),
                        f_ext, tau,
                        contact_points, num_contact_points,
                        OUT lambda, OUT tau_contact.data());

        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1);
        printf("Contact solver: %lld ns\n", duration.count());

        for (int i = 0; i < num_vel_dofs; i++) {
            tau_total[i] = tau[i] + tau_contact[i];
        }
        featherstone_forward_dynamics(art, gravity, dt, f_ext, q, u, tau_total.data(), OUT udot);
        // forward_dynamics_using_rnea(art, gravity, f_ext, q, u, tau, OUT udot);

        integrate_implicit_euler(art, dt, udot, INOUT q, INOUT u);
    }
}
}