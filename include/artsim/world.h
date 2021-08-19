//
// Created by lasagnaphil on 21. 7. 9..
//

#ifndef EOS_SCAN_TO_HUMAN_WORLD_H
#define EOS_SCAN_TO_HUMAN_WORLD_H

#define METHOD_GET_ID(TYPE, NAME, MEMBER) TYPE* get_##NAME(Id<TYPE> id) { return MEMBER.get(id); }
#define METHOD_REMOVE_ID(TYPE, NAME, MEMBER) void remove_##NAME(Id<TYPE> id) { MEMBER.release(id); }

#include <artsim/artsim.h>

namespace artsim {

struct MaterialDB {
    Arena<Material> materials;
    std::unordered_map<std::pair<Id<Material>, Id<Material>>, Material, pair_hash> material_pairs;

    void clear();

    Id<Material> add_material(real default_friction = 1.0f,
                              real default_restitution = 0.0f,
                              real default_restitution_threshold = 0.01f);
    METHOD_GET_ID(Material, material, materials)
    METHOD_REMOVE_ID(Material, material, materials)

    void set_material_pair(Id<Material> mat1_id, Id<Material> mat2_id,
                           real friction, real restitution, real restitution_threshold);

    Material get_material_pair(Id<Material> mat1_id, Id<Material> mat2_id);
};

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
    Arena<ArticulatedBody> articulated_bodies;
    MaterialDB material_db;

    Arena<CollisionMesh> col_meshes;

    btCollisionWorld* bt_collision_world = nullptr;
    btCollisionObject* bt_plane_col = nullptr;
    btOverlapFilterCallback* overlap_filter_callback = nullptr;

    WorldConfig cfg;

    std::vector<ContactPoint> contact_points;

public:
    void init(WorldConfig world_cfg);
    void destroy() {
        delete bt_collision_world;
        delete overlap_filter_callback;
        rigid_bodies.clear();
        articulated_bodies.clear();
        material_db.clear();
    }

    glm::rvec3 get_gravity() const { return cfg.gravity; }
    void set_gravity(const glm::rvec3& gravity) { cfg.gravity = gravity; }

    real get_timestep() const { return cfg.dt; }
    void set_timestep(real dt) { cfg.dt = dt; }

    Id<RigidBody> add_rigid_body(const RigidBodySpec& spec, Id<Material> mat_id) {
        auto id = rigid_bodies.make();
        auto ptr = rigid_bodies.get(id);
        ptr->init(id, spec, mat_id, bt_collision_world);
        return id;
    }

    RigidBody* get_rigid_body(Id<RigidBody> id) {
        return rigid_bodies.get(id);
    }

    bool remove_rigid_body(Id<RigidBody> id) {
        auto ptr = rigid_bodies.try_get(id);
        if (!ptr) return false;
        ptr->release(bt_collision_world);
        rigid_bodies.release(id);
        return true;
    }

    Id<RigidBody> add_plane(Id<Material> mat_id) {
        RigidBodySpec spec;
        const real inf = std::numeric_limits<real>::infinity();
        spec.mass = inf;
        spec.inertia = glmx::rsmat3x3(inf);
        spec.col_shape = CollisionShape::make_ground();
        // spec.render_shape = RenderShape::make_from_collision_shape(spec.col_shape);
        spec.global_trans = glmx::rtransform(glmx::IDENTITY);
        spec.is_static = true;
        return add_rigid_body(spec, mat_id);
    }

    Id<ArticulatedBody> add_articulated_body(const ArticulatedBodySpec& spec, Id<Material> mat_id,
                                             int col_filter_group = btBroadphaseProxy::DefaultFilter,
                                             int col_filter_mask = btBroadphaseProxy::AllFilter,
                                             bool enable_self_colisions = false) {
        auto id = articulated_bodies.make();
        auto ptr = articulated_bodies.get(id);
        ptr->init(id, spec, mat_id, bt_collision_world, col_filter_group, col_filter_mask, enable_self_colisions);
        load_collision_meshes(ptr->get_spec_mut());
        return id;
    }

    ArticulatedBody* get_articulated_body(Id<ArticulatedBody> id) {
        return articulated_bodies.get(id);
    }

    bool remove_articulated_body(Id<ArticulatedBody> id) {
        auto ptr = articulated_bodies.try_get(id);
        if (!ptr) return false;
        ptr->release(bt_collision_world);
        articulated_bodies.release(id);
        return true;
    }

    Id<Material> add_material(real default_friction = 1.0f,
                              real default_restitution = 0.0f,
                              real default_restitution_threshold = 0.01f) {
        return material_db.add_material(default_friction, default_restitution, default_restitution_threshold);
    }

    Material* get_material(Id<Material> id) {
        return material_db.get_material(id);
    }

    void remove_material(Id<Material> id) {
        return material_db.remove_material(id);
    }

    void set_material_pair(Id<Material> mat1_id, Id<Material> mat2_id,
                           real friction, real restitution, real restitution_threshold) {
        material_db.set_material_pair(mat1_id, mat2_id, friction, restitution, restitution_threshold);
    }

    Id<CollisionMesh> add_collision_mesh_bvh(const char* objfile) {
        auto id = col_meshes.make();
        auto ptr = col_meshes.get(id);
        ptr->init_bvh(objfile);
        return id;
    }

    Id<CollisionMesh> add_collision_mesh_sdf(const char* objfile, real sdf_grid_size) {
        auto id = col_meshes.make();
        auto ptr = col_meshes.get(id);
        ptr->init_sdf(objfile, sdf_grid_size);
        return id;
    }

    CollisionMesh* get_collision_mesh(Id<CollisionMesh> id) {
        return col_meshes.get(id);
    }

    bool remove_collision_mesh(Id<CollisionMesh> id) {
        auto ptr = col_meshes.try_get(id);
        if (!ptr) return false;
        col_meshes.release(id);
        return true;
    }

    void simulate(real dt);

    btCollisionWorld* get_bullet_collision_world() {
        return bt_collision_world;
    }

    std::vector<ContactPoint>& get_contact_points() { return contact_points; }
    const std::vector<ContactPoint>& get_contact_points() const { return contact_points; }

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
