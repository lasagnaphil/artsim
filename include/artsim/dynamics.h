//
// Created by Phillip Chang on 2020/09/12.
//

#ifndef ARTSIM_DYNAMICS_H
#define ARTSIM_DYNAMICS_H

#include "artsim/artsim.h"
#include "artsim/math/se3.h"
#include <queue>

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
    void jcalc(const Joint& joint, const Link& link, const T* q, OUT KinematicsData<T>& kin) {
        switch (joint.type) {
            case JointType::Revolute: {
                tscrew<T> S = Ad(link.local_joint_pose, screw(joint.revolute.axis, tvec3<T>(0)));
                kin.Tinv = big_adj(S, -q[0]) * inverse(link.local_link_pose);
                kin.S[0] = S;
                kin.v = S * q[0];
                kin.c = tscrew<T>();
            } break;
            case JointType::Prismatic: {
                tscrew<T> S = Ad(link.local_joint_pose, screw(tvec3<T>(0), joint.prismatic.dir));
                kin.Tinv = big_adj(S, -q[0]) * inverse(link.local_link_pose);
                kin.S[0] = S;
                kin.v = S * q[0];
                kin.c = tscrew<T>();
            } break;
            case JointType::Spherical: {
                glm::tvec3<T> qvec = tvec3<T>(q[0], q[1], q[2]);
                kin.Tinv = link.local_joint_pose * exp(-qvec) * inverse(link.local_joint_pose) * inverse(link.local_link_pose);

                glm::tmat3x3<T> joint_basis_vecs = glm::mat3_cast<T>(link.local_joint_pose);
                kin.S[0].w = joint_basis_vecs[0];
                kin.S[0].v = glm::cross(link.local_joint_pose.v, joint_basis_vecs[0]);
                kin.S[1].w = joint_basis_vecs[1];
                kin.S[1].v = glm::cross(link.local_joint_pose.v, joint_basis_vecs[1]);
                kin.S[2].w = joint_basis_vecs[2];
                kin.S[2].v = glm::cross(link.local_joint_pose.v, joint_basis_vecs[2]);

                kin.v = kin.S[0] * q[0] + kin.S[1] * q[1] + kin.S[2] * q[2];
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
        uint32_t joint_dof;
        bool has_parent;

        // IN
        KinematicsData<T> kin;
        tspmat<T> I;
        tscrew<T> f_ext;
        tvec3<T> q2dot;         // dof

        // PARENT
        ttransform<T> T_parent_global_inv;  // set before pass 1
        tscrew<T> f_parent; // set after pass 1

        // INTERMEDIATE VALUES
        ttransform<T> T_global_inv;

        // OUT
        tscrew<T> v;
        tscrew<T> a;
        tscrew<T> f;
        tvec3<T> tau;           // dof

        // kin must be calculated using jcalc() before this call
        void rnea_pass1() {
            if (has_parent) T_global_inv = kin.Tinv * T_parent_global_inv;
            v = Ad(kin.Tinv, v) + kin.v;
            a = Ad(kin.Tinv, a) + ad(v, kin.v) + kin.c;
            for (int i = 0; i < joint_dof; i++) {
                a += kin.S[i] * q2dot[i];
            }
            f = I * a + adT(v, I * v) - AdT(T_parent_global_inv, f_ext);
        }

        void rnea_pass2() {
            for (int i = 0; i < joint_dof; i++) {
                tau[i] = dot(kin.S[i], f[i]);
                if (has_parent) {
                    f_parent += AdT(kin.Tinv, f);
                }
            }
        }
    };

    template <class T>
    void rne_inverse_dynamics(const ArticulatedBody& art,
                              const T*__restrict q, const T*__restrict qdot, const T*__restrict q2dot,
                              const T*__restrict tau) {

        int num_joints = art.get_num_joints();
        std::vector<RecursiveNewtonEulerData<T>> data(num_joints);

        // TODO: populate data

        for (int i : art.bfs_iteration_order) {
            jcalc(art.joints[i], art.links[i], data[i].kin);
            if (i != 0) {
                data[i].T_parent_global_inv = data[art.parents[i]].T_global_inv;
            }
            data[i].rnea_pass1();
            if (i != 0) {
                data[i].f_parent = data[art.parents[i]].f;
            }
        }

        for (int j = num_joints - 1; j >= 0; j--) {
            int i = art.bfs_iteration_order[j];
            data[i].rnea_pass2();
        }

        int cur_dof = 0;
        for (int i = 0; i < num_joints; i++) {
            int num_dofs = art.joint_dofs[i];
            for (int j = 0; j < num_dofs; j++) {
                tau[cur_dof + j] = data[i].tau[j];
            }
        }
    }


    template <class T>
    struct FeatherstoneData {
        uint32_t joint_dof;     // 1 (revolute, prismatic) or 3 (spherical)
        bool has_parent;

        // IN
        KinematicsData<T> kin;
        tspmat<T> I;
        tscrew<T> f_ext;
        tvec3<T> tau;          // dof

        // PARENT
        ttransform<T> T_parent_global_inv;  // set before pass 1
        tscrew<T> v_parent;                 // set before pass 1
        tsmat6x6<T> I_a_parent;             // set before pass 2
        tscrew<T> p_a_parent;               // set before pass 2
        tscrew<T> a_parent;                 // set before pass 3

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
        tvec3<T> q2dot;         // dof

        // kin must be calculated using jcalc() before this call
        inline void featherstone_pass1() {
            if (has_parent) T_global_inv = kin.Tinv * T_parent_global_inv;
            v = Ad(kin.Tinv, v_parent) + kin.v;
            c = ad(v, kin.v) + kin.c;
            I_a = I;
            p_a = adT(v, I * v) - AdT(T_global_inv, f_ext);
        }

        inline void featherstone_pass2() {
            tsmat6x6<T> I_prime;
            tscrew<T> p_prime;

            if (joint_dof == 1) {
                U[0] = I_a * kin.S[0];
                D[0][0] = dot(kin.S[0], U[0]);
                u = tau - dot(kin.S[0], p_a);
                if (has_parent) {
                    I_prime = I_a - symmetric_cartesian_product(U[0]) / D[0][0];
                    p_prime = p_a + I_a * c + u[0] / D[0][0] * U[0];
                }
            }
            else if (joint_dof == 3) {
                mult_6x6_6x3(I_a, kin.S, OUT U);
                mult_3x6_6x3(kin.S, U, OUT D);
                u[0] = tau[0] - dot(kin.S[0], p_a);
                u[1] = tau[1] - dot(kin.S[1], p_a);
                u[2] = tau[2] - dot(kin.S[2], p_a);
                if (has_parent) {
                    tmat3x3<T> Dinv = inverse(D);
                    tscrew<T> U_Dinv[3];
                    mult_UVUt_6x3_3x3_3x6_sym(U, Dinv, OUT U_Dinv, OUT I_prime);
                    I_prime = I_a - I_prime;
                    p_prime = p_a + I_prime * c;
                    p_prime += U_Dinv[0] * u[0];
                    p_prime += U_Dinv[1] * u[1];
                    p_prime += U_Dinv[2] * u[2];
                }
            }
            I_a_parent += move_frame(I_prime, kin.Tinv);
            p_a_parent += AdT(kin.Tinv, p_prime);
        }

        inline void featherstone_pass3() {
            tscrew<T> a_p = Ad(kin.Tinv, a_parent) + c;
            if (joint_dof == 1) {
                q2dot[0] = (u[0] - dot(U[0], a_p)) / D[0][0];
                a = a_p + dot(kin.S[0], q2dot);
            }
            else if (joint_dof == 3) {
                tvec3<T> u_p;
                u_p[0] = u[0] - dot(U[0], a_p);
                u_p[1] = u[1] - dot(U[1], a_p);
                u_p[2] = u[2] - dot(U[2], a_p);
                q2dot = inverse(D) * u_p;
                a = a_p + dot(kin.S[0], q2dot) + dot(kin.S[1], q2dot) + dot(kin.S[2], q2dot);
            }
        }
    };

    template <class T>
    void featherstone_forward_dynamics(const ArticulatedBody& art,
                                       const T*__restrict q, const T*__restrict qdot, OUT T*__restrict q2dot) {

        int num_joints = art.get_num_joints();
        std::vector<FeatherstoneData<T>> data(num_joints);

        // TODO: populate data

        for (int i : art.bfs_iteration_order) {
            jcalc(art.joints[i], art.links[i], data[i].kin);
            if (i != 0) {
                data[i].T_parent_global_inv = data[art.parents[i]].T_global_inv;
                data[i].v_parent = data[art.parents[i]].v;
            }
            data[i].featherstone_pass1();
        }
        for (int j = num_joints - 1; j >= 0; j--) {
            int i = art.bfs_iteration_order[j];
            if (i != 0) {
                data[i].I_a_parent = data[art.parents[i]].I_a;
                data[i].p_a_parent = data[art.parents[i]].p_a;
            }
            data[i].featherstone_pass2();
        }
        for (int i : art.bfs_iteration_order) {
            if (i != 0) {
                data[i].a_parent = data[art.parents[i]].a;
            }
            data[i].featherstone_pass3();
        }

        int cur_dof = 0;
        for (int i = 0; i < num_joints; i++) {
            int num_dofs = art.joint_dofs[i];
            for (int j = 0; j < num_dofs; j++) {
                q2dot[cur_dof + j] = data[i].q2dot[j];
            }
        }
    }
}

#endif //ARTSIM_DYNAMICS_H
