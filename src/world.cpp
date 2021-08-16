//
// Created by lasagnaphil on 21. 7. 9..
//

#include <artsim/world.h>

#include <artsim/art_dynamics.h>
#include <artsim/math/se3.h>
#include <artsim/math/eigen.h>

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

void World::integrate_with_contacts() {
    ZoneScoped

    contact_points.clear();
    {
        ZoneNamedN(GatherContacts, "GatherContacts", true);

        auto dispatcher = bt_collision_world->getDispatcher();
        btPersistentManifold** manifolds = dispatcher->getInternalManifoldPointer();
        int num_manifolds = dispatcher->getNumManifolds();

        for (int i = 0; i < num_manifolds; i++) {
            btPersistentManifold* manifold = manifolds[i];
            int num_contacts = manifold->getNumContacts();
            if (num_contacts == 0) continue;

            const btCollisionObject* bt_body1 = manifold->getBody0();
            const btCollisionObject* bt_body2 = manifold->getBody1();
            BodyLinkId body1_id, body2_id;
            body1_id.index = bt_body1->getUserIndex();
            body1_id.generation = bt_body1->getUserIndex2();
            body2_id.index = bt_body2->getUserIndex();
            body2_id.generation = bt_body2->getUserIndex2();
            if (body1_id.index < body2_id.index) std::swap(body1_id, body2_id);
            for (int j = 0; j < num_contacts; j++) {
                auto& pt = manifold->getContactPoint(j);
                int cp_id = contact_points.size();
                ContactPoint cp;
                cp.bt_manifold = manifold;
                cp.bt_manifold_point = &pt;
                cp.pos = glmconv(pt.getPositionWorldOnB());
                cp.normal = glmconv(pt.m_normalWorldOnB);
                if (real(1) - cp.normal.x > real(1e-6)) {
                    cp.tangent1 = glm::normalize(glm::rvec3(1, 0, 0) - cp.normal.x * cp.normal);
                }
                else {
                    cp.tangent1 = glm::normalize(glm::rvec3(0, 1, 0) - cp.normal.y * cp.normal);
                }
                cp.tangent2 = glm::cross(cp.normal, cp.tangent1);
                cp.distance = pt.getDistance();
                cp.area = 0;
                cp.body1_id = body1_id;
                cp.body2_id = body2_id;
                auto global_trans1 = glmx::rtransform(cp.pos + cp.distance * cp.normal,
                                                      glm::rmat3(cp.tangent1, cp.tangent2, cp.normal));
                if (body1_id.is_articulation()) {
                    auto [art_id, lidx] = body1_id.get_articulation_id();
                    auto art = articulated_bodies.get(art_id);
                    auto joint_trans = art->get_global_joint_trans(lidx);
                    cp.body1_rel_trans = global_trans1 / joint_trans;
                }
                else {
                    auto rb = rigid_bodies.get(body1_id.get_rigid_body_id());
                    cp.body1_rel_trans = global_trans1 / glmx::rtransform(rb->pos, glm::mat3_cast(rb->rot));
                }
                auto global_trans2 = glmx::rtransform(cp.pos,
                                                      glm::rmat3(cp.tangent1, cp.tangent2, cp.normal));
                if (body2_id.is_articulation()) {
                    auto [art_id, lidx] = body2_id.get_articulation_id();
                    auto art = articulated_bodies.get(art_id);
                    auto joint_trans = art->get_global_joint_trans(lidx);
                    cp.body2_rel_trans = global_trans2 / joint_trans;
                }
                else {
                    auto rb = rigid_bodies.get(body2_id.get_rigid_body_id());
                    cp.body2_rel_trans = global_trans2 / glmx::rtransform(rb->pos, glm::mat3_cast(rb->rot));
                }
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

    switch (cfg.contact_solver_type) {
        case ContactSolverType::Proximal:
            proximal_solver(); break;
        case ContactSolverType::NCP:
            newton_solver(); break;
    }
    return;

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
        // Load collision mesh and calculate its SDF
        auto& lidx = links_to_load[i];
        auto& link = spec.links[lidx];
        auto col_mesh = col_meshes.get(link.col_shape.mesh.id);
        col_mesh->init_from_obj(link.obj_filename.c_str(),
                                link.col_shape.mesh.cell_size);

        // Calculate mass and inertia
        // Reference: https://abhilashreddy.com/writing/6/mesh_props.html
        // fmt::print("Link {}: \n", spec.names[lidx]);
        auto& obj = col_mesh->objfile;
        int num_tris = obj.triangle_vertices.size();
        auto verts = obj.vertices;
        auto mean = glm::rvec3(0);
        for (auto& v : verts) {
            mean += v;
        }
        mean /= verts.size();
        for (auto& v : verts) {
            v -= mean;
        }
        std::vector<glm::rvec3> cent(num_tris);
        std::vector<glm::rvec3> area_vec(num_tris);
        std::vector<real> area(num_tris);
        std::vector<glm::rvec3> c2f(num_tris);
        real volume = 0;
        for (int tidx = 0; tidx < num_tris; tidx++) {
            auto tri = obj.triangle_vertices[tidx];
            auto v0 = obj.vertices[tri[0]];
            auto v1 = obj.vertices[tri[1]];
            auto v2 = obj.vertices[tri[2]];
            cent[tidx] = (v0 + v1 + v2) / real(3);
            area_vec[tidx] = real(0.5) * glm::cross(v1 - v0, v2 - v0);
            area[tidx] = glm::length(area_vec[tidx]);
            c2f[tidx] = cent[tidx] * cent[tidx] * area_vec[tidx];
            volume += glm::dot(cent[tidx], area_vec[tidx]) / real(3);
        }
        volume /= 6;
        link.mass = glm::abs(volume) * link.density;
        // fmt::print("mass = {}\n", link.mass);

        auto cent_mean = glm::rvec3(0);
        for (int tidx = 0; tidx < num_tris; tidx++) {
            cent_mean += c2f[tidx];
        }
        cent_mean *= (real(0.5) / volume);
        auto p = glmx::rsmat3x3(0);
        p.xx = -volume * cent_mean.x * cent_mean.x;
        p.yy = -volume * cent_mean.y * cent_mean.y;
        p.zz = -volume * cent_mean.z * cent_mean.z;
        p.yz = volume * cent_mean.y * cent_mean.z;
        p.yy = volume * cent_mean.z * cent_mean.x;
        p.zz = volume * cent_mean.x * cent_mean.y;
        for (int tidx = 0; tidx < num_tris; tidx++) {
            p.xx += real(1.0/3.0) * cent[tidx].x * c2f[tidx].x;
            p.yy += real(1.0/3.0) * cent[tidx].y * c2f[tidx].y;
            p.zz += real(1.0/3.0) * cent[tidx].z * c2f[tidx].z;
            p.yz -= real(1.0/4.0) * (cent[tidx].y * c2f[tidx].z + cent[tidx].z * c2f[tidx].y);
            p.zx -= real(1.0/4.0) * (cent[tidx].z * c2f[tidx].x + cent[tidx].x * c2f[tidx].z);
            p.xy -= real(1.0/4.0) * (cent[tidx].x * c2f[tidx].y + cent[tidx].y * c2f[tidx].x);
        }
        link.inertia = link.density * p;
        /*
        fmt::print("inertia = {} {} {}\n"
                   "          {} {} {}\n"
                   "          {} {} {}\n",
                   link.inertia.xx, link.inertia.xy, link.inertia.zx,
                   link.inertia.xy, link.inertia.yy, link.inertia.yz,
                   link.inertia.zx, link.inertia.yz, link.inertia.zz);
                   */
    });
}

}
