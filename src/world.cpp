//
// Created by lasagnaphil on 21. 7. 9..
//

#include "artsim/world.h"

#include "artsim/art_dynamics.h"
#include "artsim/art_contacts.h"
#include "artsim/math/se3.h"

#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>

#include <Discregrid/All>

#include <queue>
#include <random>

#include <fmt/core.h>

#include <Eigen/Dense>

#include <tbb/parallel_for.h>

#include <Tracy.hpp>

using namespace glmx;

namespace artsim {

void MaterialDB::clear() {
    materials.clear();
    material_pairs.clear();
}

Id<Material>
MaterialDB::add_material(real default_friction, real default_restitution, real default_restitution_threshold) {
    auto id = materials.make();
    auto ptr = materials.get(id);
    ptr->friction = default_friction;
    ptr->restitution = default_restitution;
    ptr->restitution_threshold = default_restitution_threshold;
    return id;
}

void MaterialDB::set_material_pair(Id<Material> mat1_id, Id<Material> mat2_id, real friction, real restitution,
                                   real restitution_threshold) {
    material_pairs[std::make_pair(mat1_id, mat2_id)] = Material{friction, restitution, restitution_threshold};
}

Material MaterialDB::get_material_pair(Id<Material> mat1_id, Id<Material> mat2_id) {
    auto it = material_pairs.find({mat1_id, mat2_id});
    if (it == material_pairs.end()) {
        Material* mat1 = materials.get(mat1_id);
        Material* mat2 = materials.get(mat2_id);
        Material mat;
        mat.friction = glm::max(mat1->friction, mat2->friction);
        mat.restitution = glm::min(mat1->restitution, mat2->restitution);
        mat.restitution_threshold = glm::max(mat1->restitution_threshold, mat2->restitution_threshold);
        return mat;
    }
    else {
        return it->second;
    }
}

void World::init(WorldConfig world_cfg) {
    cfg = std::move(world_cfg);
    auto bt_collision_config = new btDefaultCollisionConfiguration;
    auto bt_dispatcher = new btCollisionDispatcher(bt_collision_config);
    auto bt_broadphase = new btDbvtBroadphase;
    this->bt_collision_world = new btCollisionWorld(bt_dispatcher, bt_broadphase, bt_collision_config);
}

void World::simulate(real dt) {
    for (auto& art : articulated_bodies) {
        art.forward_kinematics();
        art.update_colliders();
    }
    bt_collision_world->performDiscreteCollisionDetection();
    integrate_with_contacts();
}

Eigen::Matrix<real, 3, 1> glm_to_eigen(const glm::rvec3& v) {
    return Eigen::Map<Eigen::Matrix<real, 3, 1>>((real*)&v[0]);
}

void World::integrate_with_contacts() {
    ZoneScoped

    std::vector<ContactPoint> contact_points;
    {
        ZoneNamedN(GatherContacts, "GatherContacts", true);

        auto dispatcher = bt_collision_world->getDispatcher();
        btPersistentManifold** manifolds = dispatcher->getInternalManifoldPointer();
        int num_manifolds = dispatcher->getNumManifolds();

        for (int i = 0; i < num_manifolds; i++) {
            btPersistentManifold* manifold = manifolds[i];
            int num_contacts = manifold->getNumContacts();
            if (num_contacts == 0) continue;

            const btCollisionObject* body1 = manifold->getBody0();
            const btCollisionObject* body2 = manifold->getBody1();
            BodyId body1_id, body2_id;
            body1_id.index = body1->getUserIndex();
            body1_id.generation = body1->getUserIndex2();
            body2_id.index = body2->getUserIndex();
            body2_id.generation = body2->getUserIndex2();
            if (body1_id.index < body2_id.index) std::swap(body1_id, body2_id);
            for (int j = 0; j < num_contacts; j++) {
                auto& pt = manifold->getContactPoint(j);
                int cp_id = contact_points.size();
                ContactPoint cp;
                cp.bt_manifold = manifold;
                cp.bt_manifold_point = &pt;
                cp.pos = glmconv(pt.getPositionWorldOnB());
                cp.normal = glmconv(pt.m_normalWorldOnB);
                cp.depth = -pt.getDistance();
                cp.area = 0;
                cp.body1_id = body1_id;
                cp.body2_id = body2_id;
                contact_points.push_back(cp);
            }
        }
    }

    int num_contacts = contact_points.size();
    if (num_contacts == 0) {
        ZoneNamedN(IntegrateWithNoContacts, "IntegrateWithNoContacts", true);
        for (auto& art : articulated_bodies) {
            art.simulate(cfg.gravity, cfg.dt);
        }
        // TODO: Update rigid bodies
        return;
    }

    auto t1 = std::chrono::high_resolution_clock::now();

    using MatrixXr = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic>;
    using VectorXr = Eigen::Matrix<real, Eigen::Dynamic, 1>;

    std::unordered_map<BodyId, std::vector<int>> body_contact_points_map;
    std::unordered_map<Id<ArticulatedBody>, std::vector<int>> art_contact_points_map;
    std::unordered_map<Id<RigidBody>, std::vector<int>> rb_contact_points_map;

    articulated_bodies.foreach_id([&](Id<ArticulatedBody> art_id) {
        art_contact_points_map.insert({art_id, {}});
        // body_contact_points_map.insert({BodyId::from_articulation_link(art_id, 0), {}});
    });

    rigid_bodies.foreach_id([&](Id<RigidBody> rb_id) {
        rb_contact_points_map.insert({rb_id, {}});
        // body_contact_points_map.insert({BodyId::from_rigid_body(rb_id), {}});
    });

    auto insert_contact_info = [&](BodyId bid, int cidx) {
        if (bid.is_articulation()) {
            auto [art_id, art_lidx] = bid.get_articulation_id();
            art_contact_points_map[art_id].push_back(cidx);
            // bid = BodyId::from_articulation_link(art_id, 0);
        }
        else {
            auto rb_id = bid.get_rigid_body_id();
            auto rb = rigid_bodies.get(rb_id);
            if (rb->spec.is_static) return; // Don't calculate contacts if rigid body is static!
            rb_contact_points_map[rb_id].push_back(cidx);
        }
        // body_contact_points_map[bid].push_back(cidx);
    };

    {
        ZoneNamedN(InsertContactInfo, "InsertContactInfo", true);

        for (int cidx = 0; cidx < num_contacts; cidx++) {
            insert_contact_info(contact_points[cidx].body1_id, cidx);
            insert_contact_info(contact_points[cidx].body2_id, cidx);
        }
    }

    std::vector<rvec3> c(num_contacts, rvec3(0));
    std::vector<rvec3> lambda(num_contacts, rvec3(0));
    std::vector<Material> mat(num_contacts);
    dynmat<glm::rmat3> M_delassus(num_contacts, num_contacts);
    M_delassus.clear_zero();

    {
        ZoneNamedN(CalcDelassusMatrix, "CalcDelassusMatrix", true)

        int i = 0;
        for (auto& [art1_id, cidx_list] : art_contact_points_map) {
            ZoneNamedN(CalcDelassusMatrixForArt, "CalcDelassusMatrixForArt", true)
            ArticulatedBody& art1 = *articulated_bodies.get(art1_id);
            const ArticulatedBodySpec& art1_spec = art1.get_spec();
            int art1_num_joints = art1.get_num_joints();
            int art1_num_vel_dofs = art1.get_num_vel_dofs();
            int art1_num_contact_points = cidx_list.size();

            rscrew* art1_f_ext = art1.get_external_force_buf();
            real* art1_q = art1.get_pos_buf();
            real* art1_u = art1.get_vel_buf();
            real* art1_tau = art1.get_internal_force_buf();
            real* art1_q_target = art1.get_target_pos_buf();

            VectorXr art1_udot_bar(art1_num_vel_dofs);

            featherstone_forward_dynamics(art1_spec, cfg.gravity, cfg.dt,
                                          art1_f_ext, art1_q, art1_u, art1_tau, art1_q_target, OUT art1_udot_bar.data());

            VectorXr art1_u_bar = Eigen::Map<VectorXr>(art1_u, art1_num_vel_dofs) + art1_udot_bar * cfg.dt;

            MatrixXr Jc_T(art1_num_vel_dofs, 3*art1_num_contact_points);

            for (int k = 0; k < cidx_list.size(); k++) {
                int cidx = cidx_list[k];
                ContactPoint& cp = contact_points[cidx];
                bool body1_is_art1 = cp.body1_id.is_articulation() && cp.body1_id.get_articulation_id().first == art1_id;
                BodyId body1_id = body1_is_art1? cp.body1_id : cp.body2_id;
                BodyId body2_id = body1_is_art1? cp.body2_id : cp.body1_id;
                auto [_, art1_lidx] = body1_id.get_articulation_id();
                auto contact_T = rtransform(cp.pos, mat3_cast(rotation(Ez<real>(), cp.normal)));
                auto contact_rel_T = contact_T / art1.get_global_joint_trans(art1_lidx);
                rtransform* T_joint_global = art1.get_global_joint_trans_buf();
                dynmat_view<real> Jc_T_view(Jc_T.data(), art1_num_vel_dofs, 3*art1_num_contact_points);
                calc_linear_jacobian_transpose(art1.get_spec(), art1_lidx, contact_rel_T, T_joint_global,
                                               OUT Jc_T_view.slice(0, art1_num_vel_dofs, 3*k, 3));
                // lambda[cidx].x = cp->bt_manifold_point->m_appliedImpulseLateral1;
                // lambda[cidx].y = cp->bt_manifold_point->m_appliedImpulseLateral2;
                // lambda[cidx].z = cp->bt_manifold_point->m_appliedImpulse;
            }

            VectorXr tau_star = Jc_T.transpose() * art1_u_bar;

            const real beta = 0.01;
            const real slop = 5e-5;

            for (int k = 0; k < cidx_list.size(); k++) {
                int cidx = cidx_list[k];
                ContactPoint& cp = contact_points[cidx];
                bool body1_is_art1 = cp.body1_id.is_articulation() && cp.body1_id.get_articulation_id().first == art1_id;
                BodyId body1_id = body1_is_art1? cp.body1_id : cp.body2_id;
                BodyId body2_id = body1_is_art1? cp.body2_id : cp.body1_id;
                glm::rvec3 tau = make_vec3<real>(tau_star.data() + 3*k);
                tau.z -= beta / cfg.dt * glm::max<real>(cp.depth - slop, 0);
                if (glm::isnan(tau.x) || glm::isnan(tau.y) || glm::isnan(tau.z)) {
                    printf("NaN error!\n");
                }
                if (body1_is_art1) c[cidx] += tau;
                else c[cidx] -= tau;
                Id<Material> body1_mat, body2_mat;
                body1_mat = art1.get_mat_id();
                if (body2_id.is_articulation()) {
                    auto [art2_id, art2_lidx] = body2_id.get_articulation_id();
                    auto art2 = articulated_bodies.get(art2_id);
                    body2_mat = art2->get_mat_id();
                }
                else {
                    auto rb2 = rigid_bodies.get(body2_id.get_rigid_body_id());
                    body2_mat = rb2->mat_id;
                }
                mat[cidx] = material_db.get_material_pair(body1_mat, body2_mat);
            }

            MatrixXr Minv_Jc_T(art1_num_vel_dofs, 3*art1_num_contact_points);
            std::vector<real> zero_vec(art1_num_vel_dofs, 0);

            dynmat_view<real> Minv_Jc_T_view(Minv_Jc_T.data(), art1_num_vel_dofs, 3*art1_num_contact_points);
            dynmat_view<real> Jc_T_view(Jc_T.data(), art1_num_vel_dofs, 3*art1_num_contact_points);

            multiply_inverse_mass_matrix(art1_spec, cfg.dt, art1_q, Jc_T_view, OUT Minv_Jc_T_view);

            {
                ZoneNamedN(CalcDelassusMatrixMain, "CalcDelassusMatrixMain", true)
                MatrixXr M_delassus_eigen = Jc_T.transpose() * Minv_Jc_T;
                for (int k = 0; k < art1_num_contact_points; k++) {
                    for (int i = 0; i < art1_num_contact_points; i++) {
                        Eigen::Matrix<real, 3, 3> M_contact_inv_eigen = M_delassus_eigen.block<3, 3>(3*i, 3*k);
                        M_delassus(cidx_list[i], cidx_list[k]) += glm::make_mat3(M_contact_inv_eigen.data());
                    }
                }
            }
        }

        for (auto& [rb1_id, cidx_list] : rb_contact_points_map) {
            // TODO
        }
    }

    {
        ZoneNamedN(SolveContacts, "SolveContacts", true);
        std::vector<tvec3<real>> lambda_old(num_contacts);
        bool converged = false;
        const real lambda_err_tol = 1e-4;
        real lambda_err_sq;
        int iter;
        for (iter = 0; iter < cfg.max_iters; iter++) {
            std::copy(lambda.begin(), lambda.end(), lambda_old.begin());

            for (int i = 0; i < num_contacts; i++) {
                // Find solution for one contact force
                if (c[i].z > 0) {
                    lambda[i] = glm::vec3(0);
                }
                else {
                    real mu = mat[i].friction;
                    rsmat3x3 M_inv_ii = glmx::smat3_cast(M_delassus(i, i));
                    rvec3 lambda_v0 = -inverse(M_inv_ii) * c[i];
                    if (mu*mu * lambda_v0.z*lambda_v0.z >= lambda_v0.x*lambda_v0.x + lambda_v0.y*lambda_v0.y) {
                        lambda[i] = lambda_v0;
                    }
                    else {
                        tvec3<real> lambda_star = contact_projection_solver(lambda[i], M_inv_ii, c[i], mu);
                        if (glm::isnan(lambda_star[0]) || glm::isnan(lambda_star[1]) || glm::isnan(lambda_star[2])) {
                            output_log("NaN error!\n");
                        }
                        lambda[i] = lambda_star;
                    }
                }
                // Update velocities via sequential impulse
                /*
                for (int ip = 0; ip < num_contacts; ip++) {
                    if (i == ip) continue;
                    c[ip] += M_delassus(ip, i)*(lambda[i] - lambda_old[i]);
                }
                 */
                auto& cp = contact_points[i];
                std::vector<int> *cidx_list1, *cidx_list2;
                if (cp.body1_id.is_articulation()) {
                    cidx_list1 = &art_contact_points_map[cp.body1_id.get_articulation_id().first];
                }
                else {
                    cidx_list1 = &rb_contact_points_map[cp.body1_id.get_rigid_body_id()];
                }
                if (cp.body2_id.is_articulation()) {
                    cidx_list2 = &art_contact_points_map[cp.body2_id.get_articulation_id().first];
                }
                else {
                    cidx_list2 = &rb_contact_points_map[cp.body2_id.get_rigid_body_id()];
                }
                for (int cidx : *cidx_list1) {
                    if (cidx == i) continue;
                    c[cidx] += M_delassus(cidx, i)*(lambda[i] - lambda_old[i]);
                }
                for (int cidx : *cidx_list2) {
                    if (cidx == i) continue;
                    c[cidx] -= M_delassus(cidx, i)*(lambda[i] - lambda_old[i]);
                }
            }

            real lambda_diff_norm2 = 0.0;
            for (int i = 0; i < num_contacts; i++) {
                lambda_diff_norm2 += length2(lambda[i] - lambda_old[i]);
            }
            real lambda_norm2 = 0.0;
            for (int i = 0; i < num_contacts; i++) {
                lambda_norm2 += length2(lambda[i]);
            }
            if (lambda_norm2 < 1e-12) {
                lambda_err_sq = 0;
            }
            else {
                lambda_err_sq = lambda_diff_norm2 / lambda_norm2;
            }
            if (lambda_err_sq < lambda_err_tol * lambda_err_tol) {
                converged = true;
                iter++; break;
            }
        }

        real lambda_err = sqrt(lambda_err_sq);
        if (converged) {
            output_log("Contact solver converged in %d iters (error = %f)\n", iter, lambda_err);
        }
        else {
            output_log("Contact solver did not converge! (error = %f)\n", lambda_err);
        }
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1);
    output_log("Contact solver: %lld ns\n", duration.count());

    {
        ZoneNamedN(IntegrateWithContacts, "IntegrateWithContacts", true);

        articulated_bodies.foreach_id_val([&](Id<ArticulatedBody> art_id, ArticulatedBody& art) {
            int num_joints = art.get_num_joints();
            std::vector<rscrew> f_ext_tot(num_joints);
            std::copy_n(art.get_external_force_buf(), num_joints, f_ext_tot.data());
            auto it = art_contact_points_map.find(art_id);
            if (it != art_contact_points_map.end()) {
                auto& cidx_list = it->second;
                for (int cidx : cidx_list) {
                    auto& cp = contact_points[cidx];
                    bool body1_is_art1 = cp.body1_id.is_articulation() &&
                                         cp.body1_id.get_articulation_id().first == art_id;
                    BodyId body1_id = body1_is_art1? cp.body1_id : cp.body2_id;
                    BodyId body2_id = body1_is_art1? cp.body2_id : cp.body1_id;
                    auto [_, art_lidx] = body1_id.get_articulation_id();
                    auto contact_T = rtransform(cp.pos, mat3_cast(rotation(Ez<real>(), cp.normal)));
                    auto contact_rel_T = art.get_global_joint_trans(art_lidx) / contact_T;
                    f_ext_tot[art_lidx] += AdT(contact_rel_T, rscrew(rvec3(0), lambda[cidx] / cfg.dt));
                    // cp->bt_manifold_point->m_appliedImpulseLateral1 = lambda[cidx].x;
                    // cp->bt_manifold_point->m_appliedImpulseLateral2 = lambda[cidx].y;
                    // cp->bt_manifold_point->m_appliedImpulse = lambda[cidx].z;
                }
            }

            auto& spec = art.get_spec();
            real* q = art.get_pos_buf(); real* u = art.get_vel_buf(); real* udot = art.get_acc_buf();
            featherstone_forward_dynamics(spec, cfg.gravity, cfg.dt,
                                          f_ext_tot.data(), q, u,
                                          art.get_internal_force_buf(), art.get_target_pos_buf(), OUT udot);
            integrate_implicit_euler(spec, cfg.dt, udot, INOUT q, INOUT u);
        });

        rigid_bodies.foreach_id_val([&](Id<RigidBody> rb_id, RigidBody& rb) {
            // TODO
        });
    }
}

void World::load_collision_meshes(ArticulatedBodySpec &spec) {
    std::vector<int> links_to_load;
    for (int lidx = 0; lidx < spec.links.size(); lidx++) {
        auto& link = spec.links[lidx];
        if (link.col_shape.type == CollisionShape::Type::Mesh && link.col_shape.mesh.id.is_null()) {
            link.col_shape.mesh.id = col_meshes.make();
            links_to_load.push_back(lidx);
        }
    }
    tbb::parallel_for(size_t(0), links_to_load.size(), [&](size_t i) {
    // for (int i = 0; i < links_to_load.size(); i++) {
        auto& lidx = links_to_load[i];
        auto& link = spec.links[lidx];
        auto ptr = col_meshes.get(link.col_shape.mesh.id);
        ptr->init_from_obj(link.col_shape.obj_filename.c_str(),
                           link.col_shape.mesh.sdf_res);
    // }
    });

}

}
