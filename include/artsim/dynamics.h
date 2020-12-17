//
// Created by Phillip Chang on 2020/09/12.
//

#ifndef ARTSIM_DYNAMICS_H
#define ARTSIM_DYNAMICS_H

#include "artsim/artsim.h"
#include "artsim/math/se3.h"
#include "artsim/math/dynmat.h"

#include "artsim/collision.h"

#include <queue>
#include <iostream>
#include <chrono>

#include <Eigen/Dense>
#include <glm/gtc/type_ptr.hpp>

// #define SHOW_LOG

#ifdef SHOW_LOG
#define output_log(...) printf(__VA_ARGS__)
#else
#define output_log(...)
#endif

namespace artsim {
    using namespace glm;

    template <class T>
    void calc_S(const ArticulatedBody& art,
                const T*__restrict q, OUT tscrew<T>* S) {
        for (int i = 0; i < art.get_num_joints(); i++) {
            const Joint& joint = art.joints[i];
            const Link& link = art.links[i];
            uint32_t vel_start = art.joint_vel_dof_starts[i];
            uint32_t vel_dof = art.joint_vel_dofs[i];
            switch (joint.type) {
                case JointType::Floating: {
                    S[vel_start + 0] = tscrew<T>(tvec3<T>(1, 0, 0), tvec3<T>(0, 0, 0));
                    S[vel_start + 1] = tscrew<T>(tvec3<T>(0, 1, 0), tvec3<T>(0, 0, 0));
                    S[vel_start + 2] = tscrew<T>(tvec3<T>(0, 0, 1), tvec3<T>(0, 0, 0));
                    S[vel_start + 3] = tscrew<T>(tvec3<T>(0, 0, 0), tvec3<T>(1, 0, 0));
                    S[vel_start + 4] = tscrew<T>(tvec3<T>(0, 0, 0), tvec3<T>(0, 1, 0));
                    S[vel_start + 5] = tscrew<T>(tvec3<T>(0, 0, 0), tvec3<T>(0, 0, 1));
                } break;
                case JointType::Revolute: {
                    S[vel_start] = tscrew<T>(joint.revolute.axis, tvec3<T>(0));
                } break;
                case JointType::Prismatic: {
                    S[vel_start] = tscrew<T>(tvec3<T>(0), joint.prismatic.dir);
                } break;
                case JointType::Spherical: {
                    S[vel_start + 0] = tscrew<T>(tvec3<T>(1, 0, 0), tvec3<T>(0, 0, 0));
                    S[vel_start + 1] = tscrew<T>(tvec3<T>(0, 1, 0), tvec3<T>(0, 0, 0));
                    S[vel_start + 2] = tscrew<T>(tvec3<T>(0, 0, 1), tvec3<T>(0, 0, 0));
                } break;
            }
        }
    }

    template <class T>
    struct KinematicsData {
        ttransform<T> Tinv;
        tscrew<T> S[3];
        tscrew<T> v;
        tscrew<T> c;
    };

    template <class T>
    void jcalc(const Joint& joint, const Link& link,
               const T*__restrict q, const T*__restrict u, OUT KinematicsData<T>& kin) {
        switch (joint.type) {
            case JointType::Floating: {
                kin.Tinv = ttransform<T>();
                // Skip calculation of S, v, c for floating joints
            } break;
            case JointType::Revolute: {
                tscrew<T> S = tscrew<T>(joint.revolute.axis, tvec3<T>(0));
                kin.Tinv = move(S, -q[0]) * ttransform<T>(inverse(link.local_joint_pose));
                kin.S[0] = S;
                kin.v = S * u[0];
                kin.c = tscrew<T>();
            } break;
            case JointType::Prismatic: {
                tscrew<T> S = tscrew<T>(tvec3<T>(0), joint.prismatic.dir);
                kin.Tinv = move(S, -q[0]) * ttransform<T>(inverse(link.local_joint_pose));
                kin.S[0] = S;
                kin.v = S * u[0];
                kin.c = tscrew<T>();
            } break;
            case JointType::Spherical: {
                glm::tquat<T> q_inv = glm::inverse(glm::make_quat<T>(q));
                kin.Tinv = ttransform<T>(q_inv) * ttransform<T>(inverse(link.local_joint_pose));
                kin.S[0] = tscrew<T>(tvec3<T>(1, 0, 0), tvec3<T>(0, 0, 0));
                kin.S[1] = tscrew<T>(tvec3<T>(0, 1, 0), tvec3<T>(0, 0, 0));
                kin.S[2] = tscrew<T>(tvec3<T>(0, 0, 1), tvec3<T>(0, 0, 0));
                kin.v = tscrew<T>(tvec3<T>(u[0], u[1], u[2]), tvec3<T>(0));
                kin.c = tscrew<T>();
            } break;
        }
    }

    template <class T>
    void calculate_jacobian_for_local_frame(const ArticulatedBody& art,
                                      uint32_t link_idx,
                                      const artsim::ttransform<T>& T_contact,
                                      const artsim::ttransform<T>*__restrict T_joint_global,
                                      const tscrew<T>*__restrict S,
                                      OUT tscrew<T>* J_local) {
        int num_vel_dofs = art.get_num_vel_dofs();
        int num_joints = art.get_num_joints();

        std::fill_n(J_local, num_vel_dofs, tscrew<T>());

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

    // TODO: Create optimized versions of those (Possibly AVX2?)
    template <class T>
    inline void mult_6x6_6x3(const tsmat6x6<T>& A, const tscrew<T> *__restrict B, OUT tscrew<T> *__restrict C) {
        C[0] = A * B[0];
        C[1] = A * B[1];
        C[2] = A * B[2];
    }

    // Calculates U * V * U^T.
    // TODO: Create optimized versions of those (Possibly AVX2?)
    template <class T>
    inline void mult_UVUt_6x3_3x3_3x6_sym(const tscrew<T> *__restrict U, const tmat3x3<T>& V,
                                     OUT tscrew<T> *__restrict UV, OUT tsmat6x6<T>& UVUt) {
        tmat3x3<T> U_mat_w, U_mat_v;
        U_mat_w[0] = U[0].w;
        U_mat_w[1] = U[1].w;
        U_mat_w[2] = U[2].w;
        U_mat_v[0] = U[0].v;
        U_mat_v[1] = U[1].v;
        U_mat_v[2] = U[2].v;

        tmat3x3<T> UV_mat_w = U_mat_w * V;
        tmat3x3<T> UV_mat_v = U_mat_v * V;

        UV[0].w = UV_mat_w[0];
        UV[1].w = UV_mat_w[1];
        UV[2].w = UV_mat_w[2];
        UV[0].v = UV_mat_v[0];
        UV[1].v = UV_mat_v[1];
        UV[2].v = UV_mat_v[2];

        UVUt.I = smat3_cast(UV_mat_w * transpose(U_mat_w));
        UVUt.C = UV_mat_w * transpose(U_mat_v);
        UVUt.M = smat3_cast(UV_mat_v * transpose(U_mat_v));
    }

    template <class T>
    inline void mult_3x6_6x3(const tscrew<T>* A, const tscrew<T>* B, OUT tmat3x3<T>& C) {
        C[0][0] = dot(A[0], B[0]);
        C[0][1] = dot(A[0], B[1]);
        C[0][2] = dot(A[0], B[2]);
        C[1][0] = dot(A[1], B[0]);
        C[1][1] = dot(A[1], B[1]);
        C[1][2] = dot(A[1], B[2]);
        C[2][0] = dot(A[2], B[0]);
        C[2][1] = dot(A[2], B[1]);
        C[2][2] = dot(A[2], B[2]);
    }

    /*
    template <class T>
    inline void mult_6x3_3x3(const tscrew<T>* A, const tmat3x3<T>& B, OUT tscrew<T> *__restrict C) {
        // TODO
    }

    template <class T>
    inline void mult_3x3_3x6(const tmat3x3<T>& A, const tscrew<T>* B[3], OUT tscrew<T> *__restrict C) {
        // TODO
    }
     */

    template <class T>
    struct RecursiveNewtonEulerData {
        // IN
        bool is_floating_art;
        uint32_t joint_dof;
        bool has_parent;
        KinematicsData<T> kin;
        tspmat<T> I;
        tscrew<T> f_ext;
        tvec3<T> udot;         // dof

        // INTERMEDIATE VALUES
        ttransform<T> T_global_inv;

        // OUT
        tscrew<T> v;
        tscrew<T> a;
        tscrew<T> f;
        tvec3<T> tau;           // dof

        // kin must be calculated using jcalc() before this call
        void rnea_pass1() {
            if (has_parent) T_global_inv = T_global_inv * kin.Tinv;
            v = Ad(kin.Tinv, v) + kin.v;
            a = Ad(kin.Tinv, a) + ad(v, kin.v) + kin.c;
            if (!is_floating_art) {
                for (int i = 0; i < joint_dof; i++) {
                    a += kin.S[i] * udot[i];
                }
            }
            f = I * a - adT(v, I * v) - AdT(T_global_inv, f_ext);
        }

        void rnea_pass2() {
            for (int i = 0; i < joint_dof; i++) {
                tau[i] = dot(kin.S[i], f);
            }
            if (has_parent) {
                f = AdT(kin.Tinv, f);
            }
        }
    };

    template <class T>
    void rne_inverse_dynamics(const ArticulatedBody& art,
                              const T*__restrict q, const T*__restrict u, const T*__restrict udot,
                              glm::tvec3<T> gravity,
                              const tscrew<T>*__restrict f_ext,
                              OUT T*__restrict tau) {

        int num_joints = art.get_num_joints();
        std::vector<RecursiveNewtonEulerData<T>> data(num_joints);

        for (int i = 0; i < num_joints; i++) {
            uint32_t cur_pos_dof = art.joint_pos_dof_starts[i];
            uint32_t cur_vel_dof = art.joint_vel_dof_starts[i];
            int num_vel_dofs = art.joint_vel_dofs[i];
            data[i].is_floating_art = art.floating;
            data[i].joint_dof = num_vel_dofs;
            data[i].has_parent = i != 0;
            jcalc(art.joints[i], art.links[i], q + cur_pos_dof, u + cur_vel_dof, OUT data[i].kin);
            auto I0 = tspmat<T>(tsmat3x3<T>(art.links[i].inertia), glm::tvec3<T>(0), art.links[i].mass);
            data[i].I = move_frame(I0, ttransform<T>(inverse(art.links[i].local_link_pose)));
            if (f_ext) data[i].f_ext = f_ext[i];

            if (!art.floating || i != 0) {
                for (int j = 0; j < num_vel_dofs; j++) {
                    data[i].udot[j] = udot[cur_vel_dof + j];
                }
            }
        }

        for (int i : art.bfs_iteration_order) {
            if (i == 0) {
                if (art.floating) {
                    ttransform<T> T_root = ttransform<T>(make_vec3(q), glm::mat3_cast(make_quat(q + 3)));
                    data[0].T_global_inv = ttransform<T>();
                    data[0].v = make_tscrew(u);
                    data[0].a = Ad(inverse(T_root), tscrew<T>(tvec3<T>(0), -gravity));
                    data[0].f = data[0].I * data[0].a - adT(data[0].v, data[0].I * data[0].v) - data[0].f_ext;
                    continue;
                }
                else {
                    data[0].v = tscrew<T>();
                    data[0].a = tscrew<T>(tvec3<T>(0), -gravity);
                    data[0].T_global_inv = ttransform<T>();
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
            if (art.joints[i].type == JointType::Floating) {
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

    template <class T>
    struct FeatherstoneData {
        uint32_t joint_dof;     // 1 (revolute, prismatic) or 3 (spherical)
        bool has_parent;

        // IN
        KinematicsData<T> kin;
        // tspmat<T> I;
        tscrew<T> f_ext;
        tvec3<T> tau;          // dof

        // PARENT
        // tmat_transform<T> T_parent_global_inv;  // set before pass 1
        // tscrew<T> v_parent;                 // set before pass 1
        // tsmat6x6<T> I_a_parent;             // set after pass 2
        // tscrew<T> p_a_parent;               // set after pass 2
        // tscrew<T> a_parent;                 // set before pass 3

        // INTERMEDIATE VALUES
        ttransform<T> T_global_inv;
        tsmat6x6<T> I_a;
        tscrew<T> p_a;
        tscrew<T> v;
        tscrew<T> c;
        tscrew<T> U[3];         // 6*dof
        tmat3x3<T> D;           // dof*dof
        tvec3<T> u;             // dof
        tscrew<T> a;

        // OUT
        tvec3<T> udot;         // dof

        // kin must be calculated using jcalc() before this call
        inline void featherstone_pass1() {
            if (has_parent) T_global_inv = T_global_inv * kin.Tinv;
            v = Ad(kin.Tinv, v) + kin.v;
            c = ad(v, kin.v) + kin.c;
            p_a = -adT(v, I_a * v) - AdT(T_global_inv, f_ext);
        }

        inline void featherstone_pass2() {
            tsmat6x6<T> I_prime;
            tscrew<T> p_prime;

            if (joint_dof == 1) {
                U[0] = I_a * kin.S[0];
                D[0][0] = dot(kin.S[0], U[0]);
                u[0] = tau[0] - dot(kin.S[0], p_a);
            }
            else if (joint_dof == 3) {
                mult_6x6_6x3(I_a, kin.S, OUT U);
                mult_3x6_6x3(kin.S, U, OUT D);
                u[0] = tau[0] - dot(kin.S[0], p_a);
                u[1] = tau[1] - dot(kin.S[1], p_a);
                u[2] = tau[2] - dot(kin.S[2], p_a);
            }
            if (has_parent) {
                if (joint_dof == 1) {
                    tsmat6x6<T> UUt = symmetric_cartesian_product(U[0]);
                    I_prime = I_a - UUt / D[0][0];
                    p_prime = p_a + I_prime * c + u[0] / D[0][0] * U[0];
                }
                else if (joint_dof == 3) {
                    tmat3x3<T> Dinv = inverse(D);
                    tscrew<T> U_Dinv[3];
                    mult_UVUt_6x3_3x3_3x6_sym(U, Dinv, OUT U_Dinv, OUT I_prime);
                    I_prime = I_a - I_prime;
                    p_prime = p_a + I_prime * c;
                    p_prime += U_Dinv[0] * u[0];
                    p_prime += U_Dinv[1] * u[1];
                    p_prime += U_Dinv[2] * u[2];
                }
                I_a = move_frame(I_prime, kin.Tinv);
                p_a = AdT(kin.Tinv, p_prime);
            }
        }

        inline void featherstone_pass3() {
            tscrew<T> a_p = Ad(kin.Tinv, a) + c;
            if (joint_dof == 1) {
                udot[0] = (u[0] - dot(U[0], a_p)) / D[0][0];
                a = a_p + kin.S[0] * udot[0];
            }
            else if (joint_dof == 3) {
                tvec3<T> u_p;
                u_p[0] = u[0] - dot(U[0], a_p);
                u_p[1] = u[1] - dot(U[1], a_p);
                u_p[2] = u[2] - dot(U[2], a_p);
                udot = inverse(D) * u_p;
                a = a_p + kin.S[0] * udot[0] + kin.S[1] * udot[1] + kin.S[2] * udot[2];
            }
        }
    };

    template <class T>
    void featherstone_forward_dynamics(const ArticulatedBody& art,
                                       glm::tvec3<T> gravity,
                                       const tscrew<T>*__restrict f_ext,
                                       const T*__restrict q, const T*__restrict u, const T*__restrict tau,
                                       OUT T*__restrict udot) {

        int num_joints = art.get_num_joints();
        std::vector<FeatherstoneData<T>> data(num_joints);

        for (int i = 0; i < num_joints; i++) {
            uint32_t cur_pos_dof = art.joint_pos_dof_starts[i];
            uint32_t cur_vel_dof = art.joint_vel_dof_starts[i];
            int num_vel_dofs = art.joint_vel_dofs[i];
            data[i].joint_dof = art.joint_vel_dofs[i];
            data[i].has_parent = i != 0;
            jcalc(art.joints[i], art.links[i], q + cur_pos_dof, u + cur_vel_dof, OUT data[i].kin);
            auto I0 = tspmat<T>(tsmat3x3<T>(art.links[i].inertia), glm::tvec3<T>(0), art.links[i].mass);
            data[i].I_a = tsmat6x6<T>(move_frame(I0, ttransform<T>(inverse(art.links[i].local_link_pose))));
            if (f_ext) data[i].f_ext = f_ext[i];

            if (!(i == 0 && art.floating)) {
                for (int j = 0; j < num_vel_dofs; j++) {
                    data[i].tau[j] = tau[cur_vel_dof + j];
                }
            }
        }

        for (int i : art.bfs_iteration_order) {
            if (i == 0) {
                if (art.floating) {
                    data[0].T_global_inv = ttransform<T>();
                    data[0].v = make_tscrew(u);
                    data[0].p_a = -adT(data[0].v, data[0].I_a * data[0].v) - data[0].f_ext - make_tscrew(tau);
                    continue;
                }
                else {
                    data[i].T_global_inv = ttransform<T>();
                    data[i].v = tscrew<T>();
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
                    data[i].a = tscrew<T>(tvec3<T>(0), -gravity);
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
                ttransform<T> T_root = ttransform(make_vec3(q), glm::mat3_cast(make_quat(q + 3)));
                data[i].a += Ad(inverse(T_root), tscrew<T>(tvec3<T>(0), gravity));
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

    template <class T>
    inline T compute_r(T theta, glm::tvec3<T> Minv_r3, T c_z, T mu) {
        return -c_z / (Minv_r3.z / mu + Minv_r3.x * cos(theta) + Minv_r3.y * sin(theta));
    }

    template <class T>
    inline T bisection_gradient(const tsmat3x3<T>& Minv, tvec3<T> c, tvec3<T> lambda, T mu) {
        tsmat3x3<T> M = inverse(Minv);
        glm::tvec3<T> M_r3 = tvec3<T>(M.zx, M.yz, M.zz);
        glm::tvec3<T> eta = glm::cross(M_r3, glm::tvec3<T>(lambda.x, lambda.y, -mu*mu*lambda.z));
        return glm::dot(Minv * lambda + c, eta);
    }

    template <class T>
    static glm::tvec3<T> contact_bisection_solver(tvec3<T> lambda_v0, const tsmat3x3<T>& Minv, tvec3<T> c, T mu) {
        const T gamma = 1e-4;
        T theta = glm::atan(lambda_v0.y, lambda_v0.x);
        tvec3<T> Minv_r3 = tvec3<T>(Minv.zx, Minv.yz, Minv.zz);
        T r = compute_r(theta, Minv_r3, c.z, mu);
        T lambda_z = r / mu;
        tvec3<T> lambda = tvec3<T>(r*cos(theta), r*sin(theta), lambda_z);
        T D0 = bisection_gradient(Minv, c, lambda, mu);

        T theta_p_delta = glm::asin(mu*lambda_v0.z / glm::sqrt(lambda_v0.x*lambda_v0.x + lambda_v0.y*lambda_v0.y));
        T theta_p;
        if (D0 >= 0) {
            theta_p = theta - pi<T>()/2 + theta_p_delta;
        }
        else {
            theta_p = theta + pi<T>()/2 - theta_p_delta;
        }

        int iter = 0;
        tvec3<T> lambda_b;
        T theta_orig = theta;
        T theta_p_orig = theta_p;
        do {
            T theta_b = 0.5 * (theta + theta_p);
            T r_b = compute_r(theta_b, Minv_r3, c.z, mu);
            T lambda_z_b = r_b / mu;
            lambda_b = vec3(r_b*cos(theta_b), r_b*sin(theta_b), lambda_z_b);
            T grad = bisection_gradient(Minv, c, lambda_b, mu);
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

    template <class T>
    static tvec3<T> contact_projection_solver(tvec3<T> lambda, const tsmat3x3<T>& Minv, tvec3<T> c, T mu) {
        const T alpha = 0.1f;
        T r_z = alpha / Minv.zz;
        T r_t = alpha / max(Minv.xx, Minv.yy);
        tvec3<T> v = c + Minv*lambda;
        T lambda_z = max(T(0), lambda.z - r_z*v.z);
        tvec2<T> lambda_t = tvec2<T>(lambda.x - r_t*v.x, lambda.y - r_t*v.y);
        T lambda_t_len = length(lambda_t);
        if (lambda_t_len > mu*lambda_z) {
            lambda_t = mu*lambda_z*normalize(lambda_t);
        }
        return tvec3<T>(lambda_t.x, lambda_t.y, lambda_z);
    }

    template <class T>
    static std::tuple<glm::tvec3<T>, T, bool> contact_ncp_solver(tvec3<T> lambda_v0,
                                                                 const tsmat3x3<T>& Minv, tvec3<T> c, T mu, T r) {
        // Initial value for lambda
        tvec3<T> lambda = lambda_v0;
        tvec3<T> lambda_prev = lambda;

        // Damping parameter for Newton method
        const T alpha = T(0.75);

        // Newton-Raphson method with NCP formulation
        T ncp_error_sq;
        T ncp_error_sq_prev = 1e8;
        T w;

        bool success = true;

        for (int i = 0; i < 16; i++) {
            tvec3<T> v = c + Minv*lambda;

            // Calculate Jacobian of the current system
            tsmat3x3<T> J;
            {
                T a = glm::sqrt(v.x*v.x + v.y*v.y);
                T b = r*(mu*lambda.z - glm::sqrt(lambda.x*lambda.x + lambda.y*lambda.y));
                T d = glm::sqrt(a*a + b*b);
                w = (d - b) / (a + r*mu*lambda.z - d);

                J.xx = Minv.xx + w;
                J.xy = Minv.xy;
                J.yy = Minv.yy + w;
            }
            {
                T d = glm::sqrt(v.z*v.z + r*r*lambda.z*lambda.z);
                T dphi_dvn = T(1) - v.z/d;

                J.zx = dphi_dvn * Minv.zx;
                J.yz = dphi_dvn * Minv.yz;
                J.zz = r*(T(1) - r*lambda.z/d) + dphi_dvn * Minv.zz;
            }

            // NCP functions
            tvec3<T> phi;
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

            if (ncp_error_sq <= T(1e-8)) {
                // Newton method finished
                break;
            }
        }

        return {lambda, ncp_error_sq, success};
    }

    enum class ContactSolverType {
        PGS, Bisection, NCP
    };

    template <class T, ContactSolverType type>
    void solve_collision(const ArticulatedBody& art,
                         const MaterialDB& material_db,
                         glm::tvec3<T> gravity, T dt,
                         const T*__restrict q, const T*__restrict u, const T*__restrict udot_orig,
                         const tscrew<T>*__restrict f_ext, const T* tau,
                         const ContactPoint*__restrict contact_points, uint32_t num_contact_points,
                         OUT glm::tvec3<T>*__restrict out_lambda, OUT T*__restrict out_contact_forces) {

        using namespace Eigen;

        int num_vel_dofs = art.get_num_vel_dofs();
        int num_joints = art.get_num_joints();

        Eigen::Matrix<T, Dynamic, 1> u_bar(num_vel_dofs);
        for (int i = 0; i < num_vel_dofs; i++) {
            u_bar[i] = u[i] + udot_orig[i] * dt;
        }

        Eigen::Matrix<T, Dynamic, Dynamic, RowMajor> Jc(3 * num_contact_points, num_vel_dofs);
        Jc.setZero();

        std::vector<tscrew<T>> S(num_vel_dofs);
        calc_S(art, q, OUT S.data());

        std::vector<ttransform<T>> T_link_global(num_joints), T_joint_global(num_joints);
        calc_transforms(art, q, OUT T_link_global.data(), OUT T_joint_global.data());

        std::vector<tscrew<T>> J_local(num_vel_dofs);
        for (int c = 0; c < num_contact_points; c++) {
            const auto& contact_point = contact_points[c];
            if (contact_point.body1_id.is_articulation() && contact_point.body2_id.index == 0) {
                auto [art_id, art_link_idx] = contact_point.body1_id.get_articulation_id();
                calculate_jacobian_for_local_frame(art, art_link_idx,
                                                   ttransform<T>(contact_point.T_global), T_joint_global.data(), S.data(),
                                                   OUT J_local.data());
                for (int i = 0; i < num_vel_dofs; i++) {
                    Jc(3*c + 0, i) = J_local[i].v[0];
                    Jc(3*c + 1, i) = J_local[i].v[1];
                    Jc(3*c + 2, i) = J_local[i].v[2];
                }
            }
        }

        Eigen::Matrix<T, Dynamic, 1> tau_star = Jc * u_bar;

        dynmat<tsmat3x3<T>> M_contact_inv(num_contact_points, num_contact_points);

        Eigen::Matrix<T, Dynamic, Dynamic> Minv_Jc_T(num_vel_dofs, 3*num_contact_points);
        std::vector<T> zero_vec(num_vel_dofs, 0);
        for (int c = 0; c < 3*num_contact_points; c++) {
            featherstone_forward_dynamics(art, glm::tvec3<T>(0), f_ext, q, zero_vec.data(), Jc.data() + c*num_vel_dofs,
                                          OUT Minv_Jc_T.data() + c*num_vel_dofs);
        }

        for (int k = 0; k < num_contact_points; k++) {
            Eigen::Matrix<T, Dynamic, 3> Minv_Jck_T = Minv_Jc_T.middleCols(3*k, 3);
            for (int i = 0; i < num_contact_points; i++) {
                Eigen::Matrix<T, 3, Dynamic> Jci = Jc.middleRows(3*i, 3);
                Eigen::Matrix<T, 3, 3> M_contact_inv_eigen = Jci * Minv_Jck_T;
                M_contact_inv(i, k) = tsmat3x3<T>(
                        M_contact_inv_eigen(0, 0),
                        M_contact_inv_eigen(1, 1),
                        M_contact_inv_eigen(2, 2),
                        M_contact_inv_eigen(1, 2),
                        M_contact_inv_eigen(2, 0),
                        M_contact_inv_eigen(0, 1));
            }
        }

        std::vector<tvec3<T>> c(num_contact_points);
        std::vector<tvec3<T>> lambda(num_contact_points, tvec3<T>(0));

        const T beta = 0.1;
        const T slop = 1e-4;

        const T mu = 1.0;

        T alpha_min, gamma, lambda_sq_tol, ncp_error_sq_tol;
        T alpha, total_ncp_error_sq;

        if constexpr (type == ContactSolverType::PGS) {
            alpha = 0.6;
            alpha_min = 0.6;
            gamma = 1.0;
            lambda_sq_tol = 1e-6;
        }
        else if constexpr (type == ContactSolverType::Bisection) {
            alpha = 1.0;
            alpha_min = 0.7;
            gamma = 0.99;
            lambda_sq_tol = 1e-6;
        }
        else if constexpr (type == ContactSolverType::NCP) {
            alpha = 1.0;
            alpha_min = 1.0;
            gamma = 1.0;
            lambda_sq_tol = 1e-6;
            ncp_error_sq_tol = 1e-6;
            total_ncp_error_sq = 0;
        }

        for (int i = 0; i < num_contact_points; i++) {
            c[i] = make_vec3<T>(tau_star.data() + 3*i);
            c[i].z -= beta/dt*glm::max<T>(contact_points[i].depth - slop, 0);
        }

        const int max_iters = 1000;

        T lambda_norm2;
        std::vector<tvec3<T>> lambda_old(num_contact_points);
        int iter;
        for (iter = 0; iter < max_iters; iter++) {
            std::copy(lambda.begin(), lambda.end(), lambda_old.begin());

            for (int i = 0; i < num_contact_points; i++) {
                const auto& contact_point = contact_points[i];
                if (c[i].z > 0) {
                    lambda[i] = (1 - alpha)*lambda[i];
                }
                else {
                    tsmat3x3<T> M_inv_ii = M_contact_inv(i, i);
                    tvec3<T> lambda_v0 = -(inverse(M_inv_ii) * c[i]);
                    if (mu*mu * lambda_v0.z*lambda_v0.z >= lambda_v0.x*lambda_v0.x + lambda_v0.y*lambda_v0.y) {
                        lambda[i] = alpha * lambda_v0 + (1 - alpha) * lambda[i];
                    }
                    else {
                        if constexpr (type == ContactSolverType::PGS) {
                            tvec3<T> lambda_star = contact_projection_solver(lambda[i], M_inv_ii, c[i], mu);
                            lambda[i] = alpha * lambda_star + (1 - alpha) * lambda[i];
                        }
                        else if constexpr (type == ContactSolverType::Bisection) {
                            tvec3<T> lambda_star = contact_bisection_solver(lambda_v0, M_inv_ii, c[i], mu);
                            lambda[i] = alpha * lambda_star + (1 - alpha) * lambda[i];
                        }
                        else if constexpr (type == ContactSolverType::NCP) {
                            auto [lambda_star, ncp_error_sq, success] = contact_ncp_solver(lambda_v0, M_inv_ii, c[i], mu, dt);
                            lambda[i] = alpha * lambda_star + (1 - alpha) * lambda[i];
                        }
                    }
                }

                for (int ip = 0; ip < num_contact_points; ip++) {
                    if (i == ip) continue;
                    tsmat3x3<T> M_ip_i_inv = M_contact_inv(ip, i);
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
            /*
            if (total_ncp_error_sq < ncp_error_sq_tol) {
                iter++; break;
            }
            total_ncp_error_sq = T(0);
             */
        }
        if (iter == max_iters) {
            output_log("Contact solver did not converge! (error = %f)\n", sqrt(lambda_norm2));
        }
        else {
            output_log("Contact solver converged in %d iters\n", iter);
        }

        Eigen::Matrix<T, Dynamic, 1> lambda_vec = Map<Eigen::Matrix<T, Dynamic, 1>>((T*)lambda.data(), 3*num_contact_points);
        Eigen::Matrix<T, Dynamic, 1> contact_forces = Jc.transpose() * lambda_vec / dt;

        for (int c = 0; c < num_contact_points; c++) {
            if (lambda[c].z > 100) {
                output_log("Contact force too large\n");
            }
            if (lambda[c].z < 0) {
                output_log("Contact force negative\n");
            }
        }

        if (out_lambda) {
            std::memcpy(out_lambda, lambda.data(), sizeof(tvec3<T>) * num_contact_points);
        }
        if (out_contact_forces) {
            std::memcpy(out_contact_forces, contact_forces.data(), sizeof(T) * num_vel_dofs);
        }
    }

    template <class T>
    void solve_collision_ncp(const ArticulatedBody& art,
                             const MaterialDB& material_db,
                             glm::tvec3<T> gravity, T dt,
                             const T*__restrict q, const T*__restrict u, const T*__restrict udot_orig,
                             const tscrew<T>*__restrict f_ext, const T*__restrict tau,
                             const ContactPoint*__restrict contact_points, uint32_t num_contact_points,
                             OUT glm::tvec3<T>*__restrict out_lambda, OUT T*__restrict out_contact_forces) {

        using namespace Eigen;


    }

    template <class T>
    void mass_matrix_using_rnea(const ArticulatedBody& art, const T*__restrict q, OUT T*__restrict M) {
        uint32_t dof = art.get_num_vel_dofs();
        std::vector<T> u(dof, 0);
        std::vector<T> udot(dof, 0);
        std::vector<tscrew<T>> f_ext(art.get_num_joints(), tscrew<T>());
        std::vector<T> tau(dof, 0);

        udot[0] = 1;
        rne_inverse_dynamics(art, q, u.data(), udot.data(), glm::tvec3<T>(0), f_ext.data(), OUT M);
        for (int i = 1; i < dof; i++) {
            udot[i-1] = 0;
            udot[i] = 1;
            rne_inverse_dynamics(art, q, u.data(), udot.data(), glm::tvec3<T>(0), f_ext.data(), OUT M + i*dof);
        }
    }

    template <class T>
    void all_forces(const ArticulatedBody& art,
                    glm::tvec3<T> gravity,
                    const tscrew<T>*__restrict f_ext,
                    const T*__restrict q, const T*__restrict u,
                    OUT T* tau) {
        int dof = art.get_num_vel_dofs();
        std::vector<T> udot(dof, 0);
        rne_inverse_dynamics(art, q, u, udot.data(), gravity, f_ext, tau);
    }

    template <class T>
    void forward_dynamics_using_rnea(const ArticulatedBody& art,
                                     glm::tvec3<T> gravity,
                                     const tscrew<T>*__restrict f_ext,
                                     const T*__restrict q, const T*__restrict u, const T*__restrict tau,
                                     OUT T*__restrict udot) {
        using Matrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
        using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1>;

        int dof = art.get_num_vel_dofs();
        Matrix M(dof, dof);
        Vector h(dof);
        Vector b(dof);
        Vector tau_ext = Eigen::Map<const Vector>(tau, dof);
        mass_matrix_using_rnea(art, q, M.data());
        all_forces(art, gravity, f_ext, q, u, OUT h.data());
        b.noalias() = tau_ext - h;
        Eigen::Map<Vector> x = Eigen::Map<Vector>(udot, dof);
        x.noalias() = M.llt().solve(b);
    }

    template <class T>
    void integrate_implicit_euler(const ArticulatedBody& art,
                                  T dt, const T*__restrict udot,
                                  OUT T*__restrict q, OUT T*__restrict u) {
        for (int d = 0; d < art.get_num_vel_dofs(); d++) {
            u[d] += udot[d] * dt;
        }
        T* qi = q; T* qdi = u;
        for (int i = 0; i < art.get_num_joints(); i++) {
            switch (art.joints[i].type) {
                case JointType::Revolute: case JointType::Prismatic: {
                    qi[0] += qdi[0]*dt;
                } break;
                case JointType::Spherical: {
                    qi[0] += 0.5*dt*(qi[3]*qdi[0] + qi[1]*qdi[2] - qi[2]*qdi[1]);
                    qi[1] += 0.5*dt*(qi[3]*qdi[1] + qi[2]*qdi[0] - qi[0]*qdi[2]);
                    qi[2] += 0.5*dt*(qi[3]*qdi[2] + qi[0]*qdi[1] - qi[1]*qdi[0]);
                    qi[3] -= 0.5*dt*(qi[0]*qdi[0] + qi[1]*qdi[1] + qi[2]*qdi[2]);
                    T q_len = glm::sqrt(qi[0]*qi[0] + qi[1]*qi[1] + qi[2]*qi[2] + qi[3]*qi[3]);
                    qi[0] /= q_len; qi[1] /= q_len; qi[2] /= q_len; qi[3] /= q_len;
                } break;
                case JointType::Floating: {
                    // TODO: is there a more accurate way to integrate SE(3)?
                    glm::tvec3<T> p_dot = make_quat(qi+3) * make_vec3(qdi+3);
                    qi[0] += dt*p_dot[0];
                    qi[1] += dt*p_dot[1];
                    qi[2] += dt*p_dot[2];
                    qi[3] += 0.5*dt*(qi[6]*qdi[0] + qi[4]*qdi[2] - qi[5]*qdi[1]);
                    qi[4] += 0.5*dt*(qi[6]*qdi[1] + qi[5]*qdi[0] - qi[3]*qdi[2]);
                    qi[5] += 0.5*dt*(qi[6]*qdi[2] + qi[3]*qdi[1] - qi[4]*qdi[0]);
                    qi[6] -= 0.5*dt*(qi[3]*qdi[0] + qi[4]*qdi[1] + qi[5]*qdi[2]);
                    T q_len = glm::sqrt(qi[3]*qi[3] + qi[4]*qi[4] + qi[5]*qi[5] + qi[6]*qi[6]);
                    qi[3] /= q_len; qi[4] /= q_len; qi[5] /= q_len; qi[6] /= q_len;
                } break;
            }
            qi += art.joint_pos_dofs[i];
            qdi += art.joint_vel_dofs[i];
        }
    }

    template <class T>
    void calc_transforms(const ArticulatedBody& art,
                         const T*__restrict q,
                         OUT ttransform<T>* T_link_globals,
                         OUT ttransform<T>* T_joint_globals) {

        for (uint32_t i : art.bfs_iteration_order) {
            int d = art.joint_pos_dof_starts[i];
            auto& joint = art.joints[i];
            auto& link = art.links[i];
            ttransform<T> T_joint_global_parent;
            if (i == 0) {
                T_joint_global_parent = ttransform<T>();
            }
            else {
                T_joint_global_parent = T_joint_globals[art.parents[i]];
            }
            switch (joint.type) {
                case JointType::Revolute: {
                    tscrew<T> S = tscrew<T>(joint.revolute.axis, tvec3<T>(0));
                    T_joint_globals[i] = T_joint_global_parent * ttransform<T>(link.local_joint_pose) * move(S, q[d]);
                } break;
                case JointType::Prismatic: {
                    tscrew<T> S = tscrew<T>(tvec3<T>(0), joint.prismatic.dir);
                    T_joint_globals[i] = T_joint_global_parent * ttransform<T>(link.local_joint_pose) * move(S, q[d]);
                } break;
                case JointType::Spherical: {
                    glm::tquat<T> q_j = glm::make_quat<T>(q + d);
                    T_joint_globals[i] = T_joint_global_parent * ttransform<T>(link.local_joint_pose) * glm::mat3_cast(q_j);
                } break;
                case JointType::Floating: {
                    glm::tvec3<T> v_j = glm::make_vec3<T>(q + d);
                    glm::tquat<T> q_j = glm::make_quat<T>(q + d + 3);
                    T_joint_globals[i] = T_joint_global_parent * ttransform<T>(v_j, glm::mat3_cast(q_j));
                }
            }
            T_link_globals[i] = T_joint_globals[i] * ttransform<T>(link.local_link_pose);
        }
    }

    template <class T>
    Eigen::Matrix<T, 3, 3> glm_to_eigen(const glm::tmat3x3<T>& M) {
        return Eigen::Map<Eigen::Matrix<T, 3, 3>>((T*)&M[0], 3, 3).transpose();
    }

    template <class T>
    Eigen::Matrix<T, 3, 3> glm_to_eigen(const tsmat3x3<T>& M) {
        Eigen::Matrix<T, 3, 3> Me;
        Me(0, 0) = M.xx; Me(1, 1) = M.yy; Me(2, 2) = M.zz;
        Me(1, 2) = Me(2, 1) = M.yz;
        Me(2, 0) = Me(0, 2) = M.zx;
        Me(0, 1) = Me(1, 0) = M.xy;
        return Me;
    }

    template <class T>
    Eigen::Matrix<T, 6, 6> glm_to_eigen(const tsmat6x6<T>& I) {
        Eigen::Matrix<T, 6, 6> M;
        M.template block<3,3>(0, 0) = glm_to_eigen<T>(I.I);
        M.template block<3,3>(3, 0) = glm_to_eigen<T>(I.C);
        M.template block<3,3>(0, 3) = glm_to_eigen<T>(I.C).transpose();
        M.template block<3,3>(3, 3) = glm_to_eigen<T>(I.M);
        return M;
    }

    // Mass matrix calculation using the composite-rigid-body algorithm.
    template <class T>
    void mass_matrix(const ArticulatedBody& art, const T*__restrict q, OUT T*__restrict M_ptr) {
        using namespace Eigen;
        uint32_t vdof = art.get_num_vel_dofs();
        uint32_t num_joints = art.get_num_joints();
        Eigen::Map<Eigen::Matrix<T, Dynamic, Dynamic>> M(M_ptr, vdof, vdof);
        M.setZero();

        std::vector<KinematicsData<T>> kin(num_joints);
        std::vector<tsmat6x6<T>> I(num_joints);

        // Calculates H_ij = F_i^T S_j.
#define CRBA_Ft_S(idx1, idx2, k1, k2) \
M(vpos_##idx1+k1, vpos_##idx2+k2) = M(vpos_##idx2+k2, vpos_##idx1+k1) = dot(Fi[k1], kin[idx2].S[k2])

        // Calculates Fi to M.
#define CRBA_COPY_Fi_TO_M(k) \
        M(0, vpos_i + k) = M(vpos_i + k, 0) = Fi[k].w[0]; \
        M(1, vpos_i + k) = M(vpos_i + k, 1) = Fi[k].w[1]; \
        M(2, vpos_i + k) = M(vpos_i + k, 2) = Fi[k].w[2]; \
        M(3, vpos_i + k) = M(vpos_i + k, 3) = Fi[k].v[0]; \
        M(4, vpos_i + k) = M(vpos_i + k, 4) = Fi[k].v[1]; \
        M(5, vpos_i + k) = M(vpos_i + k, 5) = Fi[k].v[2];

        std::vector<ttransform<T>> T_flink(num_joints); // used when art.floating == true

        for (uint32_t i : art.bfs_iteration_order) {
            uint32_t ppos = art.joint_pos_dof_starts[i];
            uint32_t vpos = art.joint_vel_dof_starts[i];
            jcalc(art.joints[i], art.links[i], q + ppos, q + vpos, kin[i]);
            auto I0 = tspmat<T>(tsmat3x3<T>(art.links[i].inertia), glm::tvec3<T>(0), art.links[i].mass);
            I[i] = tsmat6x6<T>(move_frame(I0, ttransform<T>(inverse(art.links[i].local_link_pose))));

            if (art.floating) {
                if (i == 0) T_flink[i] = ttransform<T>();
                else T_flink[i] = kin[i].Tinv * T_flink[art.parents[i]];
            }
        }

        int l_finish = art.floating? 1 : 0;
        for (int l = num_joints-1; l >= l_finish; l--) {
            uint32_t i = art.bfs_iteration_order[l];
            uint32_t vpos_i = art.joint_vel_dof_starts[i];
            uint32_t vdof_i = art.joint_vel_dofs[i];

            if (i != 0) {
                I[art.parents[i]] += move_frame(I[i], kin[i].Tinv);
            }
            switch (vdof_i) {
                case 1: {
                    tscrew<T> Fi[1] = {I[i] * kin[i].S[0]};
                    CRBA_Ft_S(i, i, 0, 0);
                    uint32_t j = i;
                    while (j != 0) {
                        Fi[0] = AdT(kin[j].Tinv, Fi[0]);
                        j = art.parents[j];
                        uint32_t vpos_j = art.joint_vel_dof_starts[j];
                        uint32_t vdof_j = art.joint_vel_dofs[j];
                        if (vdof_j == 1) {
                            CRBA_Ft_S(i, j, 0, 0);
                        }
                        else if (vdof_j == 3) {
                            CRBA_Ft_S(i, j, 0, 0);
                            CRBA_Ft_S(i, j, 0, 1);
                            CRBA_Ft_S(i, j, 0, 2);
                        }
                    }
                    if (art.floating) {
                        Fi[0] = AdT(T_flink[i], Fi[0]);
                        CRBA_COPY_Fi_TO_M(0);
                    }
                } break;
                case 3: {
                    tscrew<T> Fi[3] = {
                            I[i] * kin[i].S[0],
                            I[i] * kin[i].S[1],
                            I[i] * kin[i].S[2],
                    };
                    CRBA_Ft_S(i, i, 0, 0);
                    CRBA_Ft_S(i, i, 0, 1);
                    CRBA_Ft_S(i, i, 0, 2);
                    // CRBA_Ft_S(i, i, 1, 0);
                    CRBA_Ft_S(i, i, 1, 1);
                    CRBA_Ft_S(i, i, 1, 2);
                    // CRBA_Ft_S(i, i, 2, 0);
                    // CRBA_Ft_S(i, i, 2, 1);
                    CRBA_Ft_S(i, i, 2, 2);
                    uint32_t j = i;
                    while (j != 0) {
                        Fi[0] = AdT(kin[j].Tinv, Fi[0]);
                        Fi[1] = AdT(kin[j].Tinv, Fi[1]);
                        Fi[2] = AdT(kin[j].Tinv, Fi[2]);
                        j = art.parents[j];
                        uint32_t vpos_j = art.joint_vel_dof_starts[j];
                        uint32_t vdof_j = art.joint_vel_dofs[j];
                        if (vdof_j == 1) {
                            CRBA_Ft_S(i, j, 0, 0);
                            CRBA_Ft_S(i, j, 1, 0);
                            CRBA_Ft_S(i, j, 2, 0);
                        }
                        else if (vdof_j == 3) {
                            CRBA_Ft_S(i, j, 0, 0);
                            CRBA_Ft_S(i, j, 1, 0);
                            CRBA_Ft_S(i, j, 2, 0);
                            CRBA_Ft_S(i, j, 0, 1);
                            CRBA_Ft_S(i, j, 1, 1);
                            CRBA_Ft_S(i, j, 2, 1);
                            CRBA_Ft_S(i, j, 0, 2);
                            CRBA_Ft_S(i, j, 1, 2);
                            CRBA_Ft_S(i, j, 2, 2);
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
            M.template block<6,6>(0, 0) = glm_to_eigen<T>(I[0]);
        }
    }

    template <class T, ContactSolverType contact_solver_type>
    void euler_step_with_collision(const ArticulatedBody& art,
                                   const MaterialDB& material_db,
                                   glm::tvec3<T> gravity, T dt,
                                   const tscrew<T>*__restrict f_ext,
                                   const T*__restrict tau,
                                   const ContactPoint* contact_points, uint32_t num_contact_points,
                                   INOUT T*__restrict q, INOUT T*__restrict u,
                                   OUT T*__restrict udot, OUT glm::tvec3<T>* lambda) {
        int num_vel_dofs = art.get_num_vel_dofs();
        int num_joints = art.get_num_joints();

        std::vector<T> udot_bar(num_vel_dofs);
        featherstone_forward_dynamics(art, gravity, f_ext, q, u, tau, OUT udot_bar.data());
        // forward_dynamics_using_rnea(art, gravity, f_ext, q, u, tau, OUT udot_bar.data());

        if (num_contact_points == 0) {
            integrate_implicit_euler(art, dt, udot_bar.data(), INOUT q, INOUT u);
        }
        else {
            auto t1 = std::chrono::high_resolution_clock::now();
            std::vector<T> tau_contact(num_vel_dofs);
            std::vector<T> tau_total(num_vel_dofs);

            solve_collision<T, contact_solver_type>(art, material_db, gravity, dt,
                                                    q, u, udot_bar.data(),
                                                    f_ext, tau,
                                                    contact_points, num_contact_points,
                                                    OUT lambda, OUT tau_contact.data());

            auto t2 = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1);
            printf("Contact solver: %lld ns\n", duration.count());

            for (int i = 0; i < num_vel_dofs; i++) {
                tau_total[i] = tau[i] + tau_contact[i];
                output_log("%f, ", tau_total[i]);
            }
            output_log("\n");
            featherstone_forward_dynamics(art, gravity, f_ext, q, u, tau_total.data(), OUT udot);
            // forward_dynamics_using_rnea(art, gravity, f_ext, q, u, tau, OUT udot);

            integrate_implicit_euler(art, dt, udot, INOUT q, INOUT u);
        }
    }

}

#endif //ARTSIM_DYNAMICS_H
