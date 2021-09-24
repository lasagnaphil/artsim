//
// Created by lasagnaphil on 21. 7. 9..
//

#ifndef EOS_SCAN_TO_HUMAN_WORLD_H
#define EOS_SCAN_TO_HUMAN_WORLD_H

#define METHOD_GET_ID(TYPE, NAME, MEMBER) TYPE* get_##NAME(Id<TYPE> id) { return MEMBER.get(id); }
#define METHOD_REMOVE_ID(TYPE, NAME, MEMBER) void remove_##NAME(Id<TYPE> id) { MEMBER.release(id); }

#include <artsim/material.h>
#include <artsim/contact_point.h>
#include <artsim/rigid_body.h>
#include <artsim/art_body.h>
#include <artsim/collision/collision_manager.h>

namespace artsim {

enum class ContactSolverType {
    Proximal, NCP
};

enum class IntegrationType {
    SemiImplicitEuler, Midpoint
};

struct WorldConfig {
    glm::rvec3 gravity = {0.0, -9.8, 0.0};
    real dt = 1.0 / 240.0;
    int max_vel_iters = 8;
    int max_pos_iters = 2;
    ContactSolverType contact_solver_type = ContactSolverType::Proximal;
    IntegrationType integration_type = IntegrationType::SemiImplicitEuler;
};

class World {
private:
    Arena<RigidBody> rigid_bodies;
    Arena<ArticulatedBodySpec> art_body_specs;
    Arena<ArticulatedBody> articulated_bodies;
    MaterialDB material_db;

    Arena<CollisionMesh> col_meshes;
    Arena<CollisionShape> col_shapes;

    Id<CollisionShape> plane_col = {};

    WorldConfig cfg;

public:
    friend class CollisionManager;

    void init(WorldConfig world_cfg);
    void destroy();

    glm::rvec3 get_gravity() const { return cfg.gravity; }
    void set_gravity(const glm::rvec3& gravity) { cfg.gravity = gravity; }

    real get_timestep() const { return cfg.dt; }
    void set_timestep(real dt) { cfg.dt = dt; }

    Id<CollisionMesh> add_collision_mesh_bvh(const char* objfile);
    Id<CollisionMesh> add_collision_mesh_sdf(const char* objfile, real sdf_grid_size);
    CollisionMesh* get_collision_mesh(Id<CollisionMesh> id) { return col_meshes.get(id); }
    bool remove_collision_mesh(Id<CollisionMesh> id) { return col_meshes.release(id); }

    Id<CollisionShape> add_ground_shape();
    Id<CollisionShape> add_box_shape(glm::rvec3 size);
    Id<CollisionShape> add_sphere_shape(real radius);
    Id<CollisionShape> add_mesh_shape_bvh(const char* objfile, glm::rvec3 scale);
    Id<CollisionShape> add_mesh_shape_sdf(const char* objfile, real cell_size, glm::rvec3 scale);
    CollisionShape* get_shape(Id<CollisionShape> id) { return col_shapes.get(id); }

    Id<RigidBody> add_rigid_body(Id<CollisionShape> shape_id, Id<Material> mat_id,
                                 real mass, glmx::rsmat3x3 inertia, glmx::rquat_transform offset_from_com,
                                 CollisionFlags collision_flags = CF_DYNAMIC_OBJECT,
                                 CollisionMask filter_group = CM_DEFAULT,
                                 CollisionMask filter_mask = CM_ALL);
    RigidBody* get_rigid_body(Id<RigidBody> id) { return rigid_bodies.get(id); }
    bool remove_rigid_body(Id<RigidBody> id) { return rigid_bodies.release(id); }

    Id<RigidBody> add_plane(Id<Material> mat_id);

    Id<ArticulatedBodySpec> add_art_body_spec(const ArticulatedBodySpec& spec) { return art_body_specs.insert(spec); }
    ArticulatedBodySpec* get_art_body_spec(Id<ArticulatedBodySpec> id) { return art_body_specs.get(id); }

    Id<ArticulatedBody> add_articulated_body(Id<ArticulatedBodySpec> spec_id, Id<Material> mat_id,
                                             CollisionFlags collision_flags = CF_DYNAMIC_OBJECT,
                                             CollisionMask filter_group = CM_DEFAULT,
                                             CollisionMask filter_mask = CM_ALL);
    ArticulatedBody* get_articulated_body(Id<ArticulatedBody> id) { return articulated_bodies.get(id); }
    bool remove_articulated_body(Id<ArticulatedBody> id) { return articulated_bodies.release(id); }

    Id<Material> add_material(real default_friction = 1.0f,
                              real default_restitution = 0.0f,
                              real default_restitution_threshold = 0.01f) {
        return material_db.add_material(default_friction, default_restitution, default_restitution_threshold);
    }
    Material* get_material(Id<Material> id) { return material_db.get_material(id); }
    void remove_material(Id<Material> id) { return material_db.remove_material(id); }

    void set_material_pair(Id<Material> mat1_id, Id<Material> mat2_id,
                           real friction, real restitution, real restitution_threshold) {
        material_db.set_material_pair(mat1_id, mat2_id, friction, restitution, restitution_threshold);
    }

    void simulate();

private:
    void load_collision_meshes(ArticulatedBodySpec& spec);
    void integrate_with_contacts();

    void proximal_solver();
    void newton_solver();

    Id<Material> get_material(BodyLinkId blid);
};

struct WorldOverlapFilterCallback : public btOverlapFilterCallback {
    artsim::World* world;

    WorldOverlapFilterCallback(artsim::World* world) : world(world) {}

    bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const override;
};

}

#undef METHOD_GET_ID
#undef METHOD_DELETE_ID


#endif //EOS_SCAN_TO_HUMAN_WORLD_H
