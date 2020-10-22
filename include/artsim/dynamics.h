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

#include <Eigen/Dense>
#include <glm/gtc/type_ptr.hpp>

#define SHOW_LOG

#ifdef SHOW_LOG
#define output_log(...) printf(__VA_ARGS__)
#else
#define output_log(...)
#endif

namespace artsim {
    using namespace glm;

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
                glm::tvec3<T> root_pos = glm::make_vec3(q);
                glm::tquat<T> root_rot = glm::make_quat(q + 3);
                kin.Tinv = inverse(ttransform<T>(root_pos, root_rot)) * ttransform<T>(inverse(link.local_link_pose));
                // S, v, c are not calculated for floating joints
            } break;
            case JointType::Revolute: {
                tscrew<T> S = Ad(ttransform<T>(link.local_joint_pose), tscrew<T>(joint.revolute.axis, tvec3<T>(0)));
                kin.Tinv = move(S, -q[0]) * ttransform<T>(inverse(link.local_link_pose));
                kin.S[0] = S;
                kin.v = S * u[0];
                kin.c = tscrew<T>();
            } break;
            case JointType::Prismatic: {
                tscrew<T> S = Ad(ttransform<T>(link.local_joint_pose), tscrew<T>(tvec3<T>(0), joint.prismatic.dir));
                kin.Tinv = move(S, -q[0]) * ttransform<T>(inverse(link.local_link_pose));
                kin.S[0] = S;
                kin.v = S * u[0];
                kin.c = tscrew<T>();
            } break;
            case JointType::Spherical: {
                glm::tquat<T> q_inv = glm::inverse(glm::make_quat<T>(q));
                ttransform<T> T_j = ttransform<T>(link.local_joint_pose);
                kin.Tinv = T_j * ttransform<T>(q_inv) * inverse(T_j) * ttransform<T>(inverse(link.local_link_pose));

                glm::tmat3x3<T> joint_basis_vecs = glm::mat3_cast<T>(T_j.q);
                kin.S[0].w = joint_basis_vecs[0];
                kin.S[0].v = glm::cross(tvec3<T>(T_j.v), joint_basis_vecs[0]);
                kin.S[1].w = joint_basis_vecs[1];
                kin.S[1].v = glm::cross(tvec3<T>(T_j.v), joint_basis_vecs[1]);
                kin.S[2].w = joint_basis_vecs[2];
                kin.S[2].v = glm::cross(tvec3<T>(T_j.v), joint_basis_vecs[2]);

                kin.v = kin.S[0] * u[0] + kin.S[1] * u[1] + kin.S[2] * u[2];
                kin.c = tscrew<T>();
            } break;
        }
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

        UVUt.I = UV_mat_w * transpose(U_mat_w);
        UVUt.C = UV_mat_w * transpose(U_mat_v);
        UVUt.M = UV_mat_v * transpose(U_mat_v);
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
            for (int i = 0; i < joint_dof; i++) {
                a += kin.S[i] * udot[i];
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
            data[i].joint_dof = num_vel_dofs;
            data[i].has_parent = i != 0;
            jcalc(art.joints[i], art.links[i], q + cur_pos_dof, u + cur_vel_dof, OUT data[i].kin);
            data[i].I = tspmat<T>(art.links[i].inertia, glm::vec3(0), art.links[i].mass);
            if (f_ext) data[i].f_ext = f_ext[i];

            if (art.joints[i].type != JointType::Floating) {
                for (int j = 0; j < num_vel_dofs; j++) {
                    data[i].udot[j] = udot[cur_vel_dof + j];
                }
            }
        }

        for (int i : art.bfs_iteration_order) {
            if (i == 0) {
                if (art.floating) {
                    data[0].a = -tscrew<T>(tvec3<T>(0), -gravity);
                    data[0].f = data[0].I * data[0].a - adT(data[0].v, data[0].I * data[0].v) - data[0].f_ext;
                    continue;
                }
                data[i].T_global_inv = ttransform<T>();
                data[i].v = tscrew<T>();
                data[i].a = tscrew<T>(tvec3<T>(0), -gravity);
            }
            else {
                data[i].T_global_inv = data[art.parents[i]].T_global_inv;
                data[i].v = data[art.parents[i]].v;
                data[i].a = data[art.parents[i]].a;
            }
            data[i].rnea_pass1();

        }

        for (int j = num_joints - 1; j >= 0; j--) {
            if (j == 0 && art.floating) continue;
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
        // ttransform<T> T_parent_global_inv;  // set before pass 1
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
            data[i].I_a = tsmat6x6<T>(art.links[i].inertia, glm::tmat3x3<T>(0), glm::tmat3x3<T>(art.links[i].mass));
            if (f_ext) data[i].f_ext = f_ext[i];

            if (art.joints[i].type != JointType::Floating) {
                for (int j = 0; j < num_vel_dofs; j++) {
                    data[i].tau[j] = tau[cur_vel_dof + j];
                }
            }
        }

        for (int i : art.bfs_iteration_order) {
            if (i == 0) {
                if (art.floating) {
                    data[0].v = make_tscrew(u);
                    data[0].p_a = -adT(data[0].v, data[0].I_a * data[0].v) - data[0].f_ext;
                    continue;
                }
                data[i].T_global_inv = ttransform<T>();
                data[i].v = tscrew<T>();
            }
            else {
                data[i].T_global_inv = data[art.parents[i]].T_global_inv;
                data[i].v = data[art.parents[i]].v;
            }
            data[i].featherstone_pass1();
        }
        for (int j = num_joints - 1; j >= 0; j--) {
            if (j == 0 && art.floating) continue;
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
                    data[0].a = -(inverse(data[0].I_a) * data[0].p_a);
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
            if (art.joints[i].type == JointType::Floating) {
                data[i].a += tscrew<T>(tvec3<T>(0), gravity);
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

    inline float compute_r(float theta, glm::vec3 Minv_r3, float c_z, float mu) {
        float r = -c_z / (Minv_r3.z / mu + Minv_r3.x * cos(theta) + Minv_r3.y * sin(theta));
        // float lambda_z = r / mu;
        // float lambda_z = (-c_z - Minv_r3.x * r * cos(theta) - Minv_r3.y * r * sin(theta)) / Minv_r3.z;
        return r;
    }

    inline float bisection_gradient(const glm::mat3& Minv, glm::vec3 c, glm::vec3 lambda, float mu) {
        glm::mat3 M = inverse(Minv);
        glm::vec3 M_r3 = glm::vec3(M[0][2], M[1][2], M[2][2]);
        glm::vec3 eta = glm::cross(M_r3, glm::vec3(lambda.x, lambda.y, -mu*mu*lambda.z));
        return glm::dot(Minv * lambda + c, eta);
        /*
        glm::vec3 Minv_bar_c1 = Minv[0] - Minv[2] * (Minv[0][2] / Minv[2][2]);
        glm::vec3 Minv_bar_c2 = Minv[1] - Minv[2] * (Minv[1][2] / Minv[2][2]);
        glm::vec3 c_bar = c - Minv[2] * (c.z / Minv[2][2]);
        return glm::dot(lambda.x * Minv_bar_c1 + lambda.y + Minv_bar_c2 + c_bar, eta);
         */
    }

    static glm::vec3 contact_bisection_solver(const glm::mat3& Minv, glm::vec3 c, float mu) {
        const float gamma = 1e-4f;
        vec3 lambda_v0 = -inverse(Minv) * c;
        float theta = glm::atan(lambda_v0.y, lambda_v0.x);
        vec3 Minv_r3 = vec3(Minv[0].z, Minv[1].z, Minv[2].z);
        float r = compute_r(theta, Minv_r3, c.z, mu);
        float lambda_z = r / mu;
        vec3 lambda = vec3(r*cos(theta), r*sin(theta), lambda_z);
        float D0 = bisection_gradient(Minv, c, lambda, mu);

        float theta_p_delta = glm::asin(mu*lambda_v0.z / glm::sqrt(lambda_v0.x*lambda_v0.x + lambda_v0.y*lambda_v0.y));
        float theta_p;
        if (D0 >= 0.0f) {
            theta_p = theta - pi<float>()/2 + theta_p_delta;
        }
        else {
            theta_p = theta + pi<float>()/2 - theta_p_delta;
        }

        int iter = 0;
        vec3 lambda_b;
        float theta_orig = theta;
        float theta_p_orig = theta_p;
        do {
            float theta_b = 0.5f * (theta + theta_p);
            float r_b = compute_r(theta_b, Minv_r3, c.z, mu);
            float lambda_z_b = r_b / mu;
            lambda_b = vec3(r_b*cos(theta_b), r_b*sin(theta_b), lambda_z_b);
            float grad = bisection_gradient(Minv, c, lambda_b, mu);
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
    void generate_contact_jacobian_with_ground(const ArticulatedBody& art,
                                               uint32_t contact_link_idx,
                                               tvec3<T> contact_pos, tvec3<T> contact_normal,
                                               const ttransform<T>* T_link_global,
                                               const KinematicsData<T>* kin,
                                               OUT T*__restrict J) {
        int num_vel_dofs = art.get_num_vel_dofs();
        int num_joints = art.get_num_joints();

        std::fill_n(J, 3*num_vel_dofs, T(0));

        tvec3<T> contact_Ey = normalize(cross(contact_normal, Ez<T>()));
        glm::tmat3x3<T> contact_mat(Ez<T>(), contact_Ey, contact_normal);
        ttransform<T> T_contact = ttransform<T>(contact_pos, glm::quat_cast<T>(contact_mat));

        int i = contact_link_idx;
        do {
            ttransform<T> T_rel = T_link_global[i] / T_contact;
            if (art.joints[i].type == JointType::Floating) {
                glm::tmat3x3<T> R = glm::mat3_cast(T_rel.q);
                for (int j = 0; j < 3; j++) {
                    glm::vec3 Jv = glm::cross(T_rel.v, R[j]);
                    J[0*num_vel_dofs + j] = Jv.x;
                    J[1*num_vel_dofs + j] = Jv.y;
                    J[2*num_vel_dofs + j] = Jv.z;
                }
                for (int j = 0; j < 3; j++) {
                    glm::vec3 Jv = R[j];
                    J[0*num_vel_dofs + j+3] = Jv.x;
                    J[1*num_vel_dofs + j+3] = Jv.y;
                    J[2*num_vel_dofs + j+3] = Jv.z;
                }
            }
            else {
                for (int j = art.joint_vel_dof_starts[i]; j < art.joint_vel_dof_starts[i+1]; j++) {
                    glm::vec3 Jv = Ad(T_rel, kin[i].S[j]).v;
                    J[0*num_vel_dofs + j] = Jv.x;
                    J[1*num_vel_dofs + j] = Jv.y;
                    J[2*num_vel_dofs + j] = Jv.z;
                }
            }
            i = art.parents[i];
        } while (i != -1);
    }

    template <class T>
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

        std::vector<KinematicsData<T>> kin(num_joints);
        for (int i = 0; i < num_joints; i++) {
            jcalc(art.joints[i], art.links[i], q + art.joint_pos_dof_starts[i], u + art.joint_vel_dof_starts[i],
                  OUT kin[i]);
        }
        std::vector<ttransform<T>> T_local(num_joints);
        std::vector<ttransform<T>> T_global(num_joints);
        calc_transforms(art, q, OUT T_local.data(), OUT T_global.data());

        for (int c = 0; c < num_contact_points; c++) {
            const auto& contact_point = contact_points[c];
            if (contact_point.body1_id.is_link && !contact_point.body2_id.is_link) {
                if (contact_point.body2_id.index == 0) {
                    uint32_t link_idx = contact_point.body1_id.link_idx;
                    generate_contact_jacobian_with_ground(art, link_idx,
                                                          tvec3<T>(contact_point.pos), tvec3<T>(contact_point.normal),
                                                          T_global.data(), kin.data(),
                                                          OUT Jc.data() + 3*c*num_vel_dofs);
                }
            }
        }

        Eigen::Matrix<T, Dynamic, 1> tau_star = Jc * u_bar;

        dynmat<tmat3x3<T>> M_contact_inv(num_contact_points, num_contact_points);

        /*
        Eigen::Matrix<T, Dynamic, Dynamic> Minv_Jc_T(num_vel_dofs, 3*num_contact_points);
        for (int c = 0; c < 3*num_contact_points; c++) {
            featherstone_forward_dynamics(art, gravity, f_ext, q, u, Jc.data() + c*num_vel_dofs,
                                          OUT Minv_Jc_T.data() + c*num_vel_dofs);
        }

        for (int k = 0; k < num_contact_points; k++) {
            Eigen::Matrix<T, Dynamic, 3> Minv_Ak_T = Minv_Jc_T.middleCols(3*k, 3);
            for (int i = 0; i < num_contact_points; i++) {
                Eigen::Matrix<T, 3, Dynamic> Jci = Jc.middleRows(3*i, 3);
                Eigen::Matrix<T, 3, 3> M_contact_inv_eigen = Jci * Minv_Ak_T;
                M_contact_inv(i, k) = glm::make_mat3(M_contact_inv_eigen.transpose().data());
            }
        }
         */

        Eigen::Matrix<T, Dynamic, Dynamic> M(num_vel_dofs, num_vel_dofs);
        mass_matrix(art, q, OUT M.data());
        Eigen::Matrix<T, Dynamic, Dynamic> M_inv = M.inverse();
        for (int k = 0; k < num_contact_points; k++) {
            Eigen::Matrix<T, Dynamic, 3> Minv_Jck_T = M_inv * Jc.middleRows(3*k, 3).transpose();
            for (int i = 0; i < num_contact_points; i++) {
                Eigen::Matrix<T, 3, 3> M_inv_ii = Jc.middleRows(3*i, 3) * Minv_Jck_T;
                M_contact_inv(i, i) = glm::make_mat3(M_inv_ii.data());
            }
        }

        std::vector<tvec3<T>> c(num_contact_points);
        std::vector<tvec3<T>> lambda(num_contact_points, tvec3<T>(0));

        const T beta = 0.1;
        const T slop = 1e-4;

        T alpha = 0.6;
        const T alpha_min = 0.6;
        const T gamma = 1.0;
        const T mu = 1.0;

        for (int i = 0; i < num_contact_points; i++) {
            c[i] = make_vec3<T>(tau_star.data() + 3*i) - beta/dt*glm::max<T>(contact_points[i].depth - slop, 0) * Ez<T>();
        }

        const int max_iters = 64;

        T lambda_norm2;
        std::vector<tvec3<T>> lambda_old(num_contact_points);
        int iter;
        for (iter = 0; iter < max_iters; iter++) {
            std::copy(lambda.begin(), lambda.begin() + num_contact_points, lambda_old.begin());

            for (int i = 0; i < num_contact_points; i++) {
                const auto& contact_point = contact_points[i];
                if (c[i].z > 0) {
                    lambda[i] = (1 - alpha)*lambda[i];
                }
                else {
                    tmat3x3<T> M_inv_ii = M_contact_inv(i, i);
                    tmat3x3<T> M_ii = inverse(M_inv_ii);
                    tvec3<T> lambda_v0 = -M_ii * c[i];
                    if (mu*mu * lambda_v0.z*lambda_v0.z >= lambda_v0.x*lambda_v0.x + lambda_v0.y*lambda_v0.y) {
                        lambda[i] = alpha * lambda_v0 + (1 - alpha) * lambda[i];
                    }
                    else {
                        tvec3<T> lambda_star = contact_bisection_solver(M_inv_ii, c[i], mu);
                        // vec3 lambda_star = contact_projection_solver(lambda[i], M_inv_ii, c[i], mu);
                        lambda[i] = alpha * lambda_star + (1 - alpha) * lambda[i];
                    }
                }

                for (int ip = 0; ip < num_contact_points; ip++) {
                    if (i == ip) continue;
                    tmat3x3<T> M_ip_i_inv = M_contact_inv(ip, i);
                    c[ip] += M_ip_i_inv*(lambda[i] - lambda_old[i]);
                }
            }
            alpha = alpha_min + gamma * (alpha - alpha_min);
            lambda_norm2 = 0.0;
            for (int i = 0; i < num_contact_points; i++) {
                lambda_norm2 += length2(lambda[i] - lambda_old[i]);
            }
            if (lambda_norm2 < 1e-6) {
                iter++;
                break;
            }
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
        mass_matrix(art, q, M.data());
        // std::cout << M << std::endl;
        all_forces(art, gravity, f_ext, q, u, OUT h.data());
        b.noalias() = tau_ext - h;
        // std::cout << b << std::endl;
        Eigen::Map<Vector> x = Eigen::Map<Vector>(udot, dof);
        x.noalias() = M.llt().solve(b);
        // std::cout << x << std::endl;
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
                    float q_len = sqrt(qi[0]*qi[0] + qi[1]*qi[1] + qi[2]*qi[2] + qi[3]*qi[3]);
                    qi[0] /= q_len; qi[1] /= q_len; qi[2] /= q_len; qi[3] /= q_len;
                } break;
                case JointType::Floating: {
                    // TODO: is there a more accurate way to integrate SE(3)?
                    qi[0] += dt*qdi[3];
                    qi[1] += dt*qdi[4];
                    qi[2] += dt*qdi[5];
                    qi[3] += 0.5*dt*(qi[6]*qdi[0] + qi[4]*qdi[2] - qi[5]*qdi[1]); // x
                    qi[4] += 0.5*dt*(qi[6]*qdi[1] + qi[5]*qdi[0] - qi[3]*qdi[2]); // y
                    qi[5] += 0.5*dt*(qi[6]*qdi[2] + qi[3]*qdi[1] - qi[4]*qdi[0]); // z
                    qi[6] -= 0.5*dt*(qi[3]*qdi[0] + qi[4]*qdi[1] + qi[5]*qdi[2]); // w
                    float q_len = sqrt(qi[3]*qi[3] + qi[4]*qi[4] + qi[5]*qi[5] + qi[6]*qi[6]);
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
                         OUT ttransform<T>* T_link_locals,
                         OUT ttransform<T>* T_link_globals) {

        for (uint32_t i = 0; i < art.get_num_joints(); i++) {
            int d = art.joint_pos_dof_starts[i];
            auto& joint = art.joints[i];
            auto& link = art.links[i];
            switch (joint.type) {
                case JointType::Revolute: {
                    tscrew<T> S = Ad(ttransform<T>(link.local_joint_pose), tscrew<T>(joint.revolute.axis, tvec3<T>(0)));
                    T_link_locals[i] = ttransform<T>(link.local_link_pose) * move(S, q[d]);
                } break;
                case JointType::Prismatic: {
                    tscrew<T> S = Ad(ttransform<T>(link.local_joint_pose), tscrew<T>(tvec3<T>(0), joint.prismatic.dir));
                    T_link_locals[i] = ttransform<T>(link.local_link_pose) * move(S, q[d]);
                } break;
                case JointType::Spherical: {
                    glm::tquat<T> q_j = glm::make_quat<T>(q + d);
                    ttransform<T> T_j = ttransform<T>(link.local_joint_pose);
                    T_link_locals[i] = ttransform<T>(link.local_link_pose) * T_j * ttransform<T>(q_j) * inverse(T_j);
                } break;
                case JointType::Floating: {
                    glm::tvec3<T> v_j = glm::make_vec3<T>(q + d);
                    glm::tquat<T> q_j = glm::make_quat<T>(q + d + 3);
                    T_link_locals[i] = ttransform<T>(v_j, q_j);
                }
            }
        }

        for (uint32_t i : art.bfs_iteration_order) {
            if (i == 0) {
                T_link_globals[i] = T_link_locals[i];
            }
            else {
                T_link_globals[i] = T_link_globals[art.parents[i]] * T_link_locals[i];
            }
        }
    }

    template <class T>
    Eigen::Matrix<T, 3, 3> glm_to_eigen(glm::tmat3x3<T>& M) {
        return Eigen::Map<Eigen::Matrix<T, 3, 3>>(glm::value_ptr<T>(M), 3, 3).transpose();
    }

    // Mass matrix calculation using the composite-rigid-body algorithm.
    template <class T>
    void mass_matrix(const ArticulatedBody& art, const T*__restrict q, OUT T*__restrict M) {
        using namespace Eigen;
        uint32_t vdof = art.get_num_vel_dofs();
        uint32_t num_joints = art.get_num_joints();
        Eigen::Map<Eigen::Matrix<T, Dynamic, Dynamic>> H(M, vdof, vdof);
        H.setZero();

        std::vector<KinematicsData<T>> kin(num_joints);
        std::vector<tsmat6x6<T>> I(num_joints);

        if (art.floating) {
            std::vector<ttransform<T>> T_flink(num_joints);
            for (uint32_t i : art.bfs_iteration_order) {
                uint32_t ppos = art.joint_pos_dof_starts[i];
                uint32_t vpos = art.joint_vel_dof_starts[i];
                jcalc(art.joints[i], art.links[i], q + ppos, q + vpos, kin[i]);
                I[i] = tsmat6x6<T>(art.links[i].inertia, glm::tmat3x3<T>(0), glm::tmat3x3<T>(art.links[i].mass));

                if (i == 0) T_flink[i] = ttransform<T>();
                else T_flink[i] = kin[i].Tinv * T_flink[art.parents[i]];
            }

            H.template block<3,3>(0, 0) = glm_to_eigen<T>(I[0].I);
            H.template block<3,3>(3, 0) = glm_to_eigen<T>(I[0].C);
            H.template block<3,3>(0, 3) = glm_to_eigen<T>(I[0].C).transpose();
            H.template block<3,3>(3, 3) = glm_to_eigen<T>(I[0].M);

            for (int l = num_joints-1; l >= 0; l--) {
                uint32_t i = art.bfs_iteration_order[l];
                if (i == 0 && art.joints[i].type == JointType::Floating) break;

                uint32_t vpos_i = art.joint_vel_dof_starts[i];
                uint32_t vdof_i = art.joint_vel_dofs[i];
                if (i != 0) {
                    I[art.parents[i]] += move_frame(I[i], kin[i].Tinv);
                }
                switch (vdof_i) {
                    case 1: {
                        tscrew<T> Fi = I[i] * kin[i].S[0];
                        H(vpos_i, vpos_i) = dot(kin[i].S[0], Fi);
                        uint32_t j = i;
                        while (j != 0) {
                            Fi = AdT(kin[j].Tinv, Fi);
                            j = art.parents[j];
                            uint32_t vpos_j = art.joint_vel_dof_starts[j];
                            uint32_t vdof_j = art.joint_vel_dofs[j];
                            if (vdof_j == 1) {
                                H(vpos_i, vpos_j+0) = H(vpos_j+0, vpos_i) = dot(Fi, kin[j].S[0]);
                            }
                            else if (vdof_j == 3) {
                                H(vpos_i, vpos_j+0) = H(vpos_j+0, vpos_i) = dot(Fi, kin[j].S[0]);
                                H(vpos_i, vpos_j+1) = H(vpos_j+1, vpos_i) = dot(Fi, kin[j].S[1]);
                                H(vpos_i, vpos_j+2) = H(vpos_j+2, vpos_i) = dot(Fi, kin[j].S[2]);
                            }
                        }
                        Fi = AdT(T_flink[i], Fi);
                        H(0, vpos_i) = Fi.w[0];
                        H(1, vpos_i) = Fi.w[1];
                        H(2, vpos_i) = Fi.w[2];
                        H(3, vpos_i) = Fi.v[0];
                        H(4, vpos_i) = Fi.v[1];
                        H(5, vpos_i) = Fi.v[2];
                    } break;
                    case 3: {
                        tscrew<T> Fi[3];
                        Fi[0] = I[i] * kin[i].S[0];
                        Fi[1] = I[i] * kin[i].S[1];
                        Fi[2] = I[i] * kin[i].S[2];
                        H(vpos_i + 0, vpos_i + 0) = dot(kin[i].S[0], Fi[0]);
                        H(vpos_i + 0, vpos_i + 1) = dot(kin[i].S[0], Fi[1]);
                        H(vpos_i + 0, vpos_i + 2) = dot(kin[i].S[0], Fi[2]);
                        H(vpos_i + 1, vpos_i + 0) = dot(kin[i].S[1], Fi[0]);
                        H(vpos_i + 1, vpos_i + 1) = dot(kin[i].S[1], Fi[1]);
                        H(vpos_i + 1, vpos_i + 2) = dot(kin[i].S[1], Fi[2]);
                        H(vpos_i + 2, vpos_i + 0) = dot(kin[i].S[2], Fi[0]);
                        H(vpos_i + 2, vpos_i + 1) = dot(kin[i].S[2], Fi[1]);
                        H(vpos_i + 2, vpos_i + 2) = dot(kin[i].S[2], Fi[2]);
                        uint32_t j = i;
                        while (j != 0) {
                            Fi[0] = AdT(kin[j].Tinv, Fi[0]);
                            Fi[1] = AdT(kin[j].Tinv, Fi[1]);
                            Fi[2] = AdT(kin[j].Tinv, Fi[2]);
                            j = art.parents[j];
                            uint32_t vpos_j = art.joint_vel_dof_starts[j];
                            uint32_t vdof_j = art.joint_vel_dofs[j];
                            if (vdof_j == 1) {
#define CRBA_MACRO(k1) \
                            H(vpos_i+k1, vpos_j) = H(vpos_j, vpos_i+k1) = dot(Fi[k1], kin[j].S[0])
                                CRBA_MACRO(0);
                                CRBA_MACRO(1);
                                CRBA_MACRO(2);
#undef CRBA_MACRO
                            }
                            else if (vdof_j == 3) {
#define CRBA_MACRO(k1, k2) \
                            H(vpos_i+k1, vpos_j+k2) = H(vpos_j+k2, vpos_i+k1) = dot(Fi[k1], kin[j].S[k2])
                                CRBA_MACRO(0, 0);
                                CRBA_MACRO(1, 0);
                                CRBA_MACRO(2, 0);
                                CRBA_MACRO(0, 1);
                                CRBA_MACRO(1, 1);
                                CRBA_MACRO(2, 1);
                                CRBA_MACRO(0, 2);
                                CRBA_MACRO(1, 2);
                                CRBA_MACRO(2, 2);
#undef CRBA_MACRO
                            }
                        }
                        for (int k = 0; k < 3; k++) {
                            Fi[k] = AdT(T_flink[i], Fi[k]);
                            H(0, vpos_i + k) = Fi[k].w[0];
                            H(1, vpos_i + k) = Fi[k].w[1];
                            H(2, vpos_i + k) = Fi[k].w[2];
                            H(3, vpos_i + k) = Fi[k].v[0];
                            H(4, vpos_i + k) = Fi[k].v[1];
                            H(5, vpos_i + k) = Fi[k].v[2];
                        }
                    } break;
                    default: break;
                }
            }
            H.bottomLeftCorner(vdof-6, 6) = H.topRightCorner(6, vdof-6).transpose();
        }
        else {
            for (uint32_t i : art.bfs_iteration_order) {
                uint32_t ppos = art.joint_pos_dof_starts[i];
                uint32_t vpos = art.joint_vel_dof_starts[i];
                jcalc(art.joints[i], art.links[i], q + ppos, q + vpos, kin[i]);
                I[i] = tsmat6x6<T>(art.links[i].inertia, glm::tmat3x3<T>(0), glm::tmat3x3<T>(art.links[i].mass));
            }
            for (int l = num_joints-1; l >= 0; l--) {
                uint32_t i = art.bfs_iteration_order[l];
                uint32_t vpos_i = art.joint_vel_dof_starts[i];
                uint32_t vdof_i = art.joint_vel_dofs[i];
                if (i != 0) {
                    I[art.parents[i]] += move_frame(I[i], kin[i].Tinv);
                }
                switch (vdof_i) {
                    case 1: {
                        tscrew<T> F;
                        F = I[i] * kin[i].S[0];
                        M[vpos_i * vdof + vpos_i] = dot(kin[i].S[0], F);
                        uint32_t j = i;
                        while (j != 0) {
                            F = AdT(kin[j].Tinv, F);
                            j = art.parents[j];
                            uint32_t vpos_j = art.joint_vel_dof_starts[j];
                            uint32_t vdof_j = art.joint_vel_dofs[j];
                            if (vdof_j == 1) {
                                H(vpos_i, vpos_j+0) = H(vpos_j+0, vpos_i) = dot(F, kin[j].S[0]);
                            }
                            else if (vdof_j == 3) {
                                H(vpos_i, vpos_j+0) = H(vpos_j+0, vpos_i) = dot(F, kin[j].S[0]);
                                H(vpos_i, vpos_j+1) = H(vpos_j+1, vpos_i) = dot(F, kin[j].S[1]);
                                H(vpos_i, vpos_j+2) = H(vpos_j+2, vpos_i) = dot(F, kin[j].S[2]);
                            }
                        }
                    } break;
                    case 3: {
                        tscrew<T> F[3];
                        F[0] = I[i] * kin[i].S[0];
                        F[1] = I[i] * kin[i].S[1];
                        F[2] = I[i] * kin[i].S[2];
                        H(vpos_i + 0, vpos_i + 0) = dot(kin[i].S[0], F[0]);
                        H(vpos_i + 0, vpos_i + 1) = dot(kin[i].S[0], F[1]);
                        H(vpos_i + 0, vpos_i + 2) = dot(kin[i].S[0], F[2]);
                        H(vpos_i + 1, vpos_i + 0) = dot(kin[i].S[1], F[0]);
                        H(vpos_i + 1, vpos_i + 1) = dot(kin[i].S[1], F[1]);
                        H(vpos_i + 1, vpos_i + 2) = dot(kin[i].S[1], F[2]);
                        H(vpos_i + 2, vpos_i + 0) = dot(kin[i].S[2], F[0]);
                        H(vpos_i + 2, vpos_i + 1) = dot(kin[i].S[2], F[1]);
                        H(vpos_i + 2, vpos_i + 2) = dot(kin[i].S[2], F[2]);
                        uint32_t j = i;
                        while (j != 0) {
                            F[0] = AdT(kin[j].Tinv, F[0]);
                            F[1] = AdT(kin[j].Tinv, F[1]);
                            F[2] = AdT(kin[j].Tinv, F[2]);
                            j = art.parents[j];
                            uint32_t vpos_j = art.joint_vel_dof_starts[j];
                            uint32_t vdof_j = art.joint_vel_dofs[j];
                            if (vdof_j == 1) {
#define CRBA_MACRO(k1) \
                            H(vpos_i+k1, vpos_j) = H(vpos_j, vpos_i+k1) = dot(F[k1], kin[j].S[0])
                                CRBA_MACRO(0);
                                CRBA_MACRO(1);
                                CRBA_MACRO(2);
#undef CRBA_MACRO
                            }
                            else if (vdof_j == 3) {
#define CRBA_MACRO(k1, k2) \
                            H(vpos_i+k1, vpos_j+k2) = H(vpos_j+k2, vpos_i+k1) = dot(F[k1], kin[j].S[k2])
                                CRBA_MACRO(0, 0);
                                CRBA_MACRO(1, 0);
                                CRBA_MACRO(2, 0);
                                CRBA_MACRO(0, 1);
                                CRBA_MACRO(1, 1);
                                CRBA_MACRO(2, 1);
                                CRBA_MACRO(0, 2);
                                CRBA_MACRO(1, 2);
                                CRBA_MACRO(2, 2);
#undef CRBA_MACRO
                            }


                        }
                    } break;
                    default: break;
                }
            }
        }

    }

    template <class T>
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

        if (num_contact_points == 0) {
            integrate_implicit_euler(art, dt, udot_bar.data(), INOUT q, INOUT u);
        }
        else {
            std::vector<T> tau_contact(num_vel_dofs);

            solve_collision(art, material_db, gravity, dt,
                            q, u, udot_bar.data(),
                            f_ext, tau,
                            contact_points, num_contact_points,
                            OUT lambda, OUT tau_contact.data());
            featherstone_forward_dynamics(art, gravity, f_ext, q, u, tau, OUT udot);

            integrate_implicit_euler(art, dt, udot, INOUT q, INOUT u);
        }
    }

}

#endif //ARTSIM_DYNAMICS_H
