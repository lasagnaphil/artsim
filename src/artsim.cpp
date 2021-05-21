//
// Created by lasagnaphil on 20. 5. 19..
//

#include "artsim/artsim.h"
#include "artsim/art_dynamics.h"
#include "artsim/art_contacts.h"
#include "artsim/math/se3.h"

#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>

#include <queue>
#include <random>

using namespace artsim;
using namespace glmx;

real CollisionShape::mass(real density) {
    switch (type) {
        case Type::Box: return density * scale.x * scale.y * scale.z;
        case Type::Sphere: return real(4.0 / 3.0) * glm::pi<real>() * scale.x * scale.y * scale.z;
        case Type::Mesh: {
            return real(1);
            // TODO: calculate proper mass
            /*
            real V = 0;
            auto& obj = *mesh.ob;
            for (auto tri : obj.triangle_vertices) {
                auto v0 = obj.vertices[tri[0]];
                auto v1 = obj.vertices[tri[1]];
                auto v2 = obj.vertices[tri[2]];
                V += glm::determinant(glm::mat3(v0, v1, v2));
            }
            V *= (density / 6);
            V = glm::abs(V);
            return V;
             */
        }
        default: return real(0);
    }
}

tsmat3x3<real> CollisionShape::inertia(real density) {
    switch (type) {
        case Type::Box: {
            const glm::tvec3<real>& s = scale;
            glm::vec3 I = mass(density) * glm::tvec3<real>(s.y*s.y + s.z*s.z, s.z*s.z + s.x*s.x, s.x*s.x + s.y*s.y) / real(12);
            return tsmat3x3<real>(I.x, I.y, I.z, 0, 0, 0);
        }
        case Type::Sphere: {
            real r = scale.x;
            glm::vec3 I = real(0.4) * mass(density) * glm::tvec3<real>(r*r);
            return tsmat3x3<real>(I.x, I.y, I.z, 0, 0, 0);
        }
        case Type::Mesh: {
            // TODO: Calculate proper inertia
            return tsmat3x3<real>(1, 1, 1, 0, 0, 0);
        }
        default: return tsmat3x3<real>(0, 0, 0, 0, 0, 0);
    }
}

CollisionShape CollisionShape::make_ground() {
    CollisionShape shape;
    shape.type = CollisionShape::Type::Ground;
    shape.bt_shape = new btStaticPlaneShape(btVector3(0, 0, 0), 0);
    return shape;
}

CollisionShape CollisionShape::make_box(glm::vec3 size) {
    CollisionShape shape;
    shape.type = CollisionShape::Type::Box;
    shape.scale = size;
    shape.bt_shape = new btBoxShape(btconv(0.5f * size));
    return shape;
}

CollisionShape CollisionShape::make_sphere(real radius) {
    CollisionShape shape;
    shape.type = CollisionShape::Type::Sphere;
    shape.scale = glm::rvec3(radius, radius, radius);
    shape.bt_shape = new btSphereShape(radius);
    return shape;
}

CollisionShape CollisionShape::make_mesh(const tinyobj::attrib_t* attrib, const tinyobj::shape_t* shapes, int num_shapes, glm::rvec3 scale) {
    CollisionShape shape;
    shape.type = CollisionShape::Type::Mesh;
    shape.scale = scale;
    // TODO: Allocate these separately
    shape.mesh.attrib = new tinyobj::attrib_t(*attrib);
    shape.mesh.shapes = new tinyobj::shape_t[num_shapes];
    for (int i = 0; i < num_shapes; i++) {
        shape.mesh.shapes[i] = shapes[i];
    }
    shape.mesh.num_shapes = num_shapes;
    // TODO: Create Bullet ConcaveMesh
    shape.bt_shape = nullptr;
    return shape;
}

RenderShape RenderShape::make_from_collision_shape(const CollisionShape& col) {
    switch (col.type) {
        case CollisionShape::Type::Ground:
            printf("Cannot make render shape for ground\n");
            return RenderShape {};
        case CollisionShape::Type::Box:
            return RenderShape::make_box(col.scale);
        case CollisionShape::Type::Sphere:
            return RenderShape::make_sphere(col.scale.x);
        case CollisionShape::Type::Mesh:
            return RenderShape::make_mesh(col.mesh.attrib, col.mesh.shapes, col.mesh.num_shapes, col.scale);
        default:
            return RenderShape {};
    }
}

RenderShape RenderShape::make_box(glm::vec3 size) {
    RenderShape shape;
    shape.type = RenderShape::Type::Box;
    shape.scale = size;
    return shape;
}

RenderShape RenderShape::make_sphere(float radius) {
    RenderShape shape;
    shape.type = RenderShape::Type::Sphere;
    shape.scale = glm::vec3(radius, radius, radius);
    return shape;
}

RenderShape RenderShape::make_mesh(const tinyobj::attrib_t* attrib, const tinyobj::shape_t* shapes, int num_shapes,
                                   glm::vec3 scale) {
    RenderShape shape;
    shape.type = RenderShape::Type::Mesh;
    // TODO: Allocate these separately
    shape.mesh.attrib = new tinyobj::attrib_t(*attrib);
    shape.mesh.shapes = new tinyobj::shape_t[num_shapes];
    for (int i = 0; i < num_shapes; i++) {
        shape.mesh.shapes[i] = shapes[i];
    }
    shape.mesh.num_shapes = num_shapes;
    shape.scale = scale;
    return shape;
}

Link Link::create(const tsmat3x3<real>& inertia, real mass,
                  CollisionShape col_shape,
                  ttransform<real> local_joint_pose, ttransform<real> local_link_pose,
                  int parent_idx, Id<Material> mat_id) {
    Link link;
    link.inertia = inertia;
    link.mass = mass;
    link.col_shape = col_shape;
    link.render_shape = RenderShape::make_from_collision_shape(col_shape);
    link.local_joint_pose = local_joint_pose;
    link.local_link_pose = local_link_pose;
    link.parent_idx = parent_idx;
    link.mat_id = mat_id;
    auto I0 = tspmat<real>(tsmat3x3<real>(link.inertia), glm::tvec3<real>(0), link.mass);
    link.I_j = inv_transform(I0, ttransform<real>(inverse(link.local_link_pose)));
    return link;
}

Link Link::create(const tsmat3x3<real>& inertia, real mass,
                  CollisionShape col_shape, RenderShape render_shape,
                  ttransform<real> local_joint_pose, ttransform<real> local_link_pose,
                  int parent_idx, Id<Material> mat_id) {
    Link link;
    link.inertia = inertia;
    link.mass = mass;
    link.col_shape = col_shape;
    link.render_shape = render_shape;
    link.local_joint_pose = local_joint_pose;
    link.local_link_pose = local_link_pose;
    link.parent_idx = parent_idx;
    link.mat_id = mat_id;
    auto I0 = tspmat<real>(tsmat3x3<real>(link.inertia), glm::tvec3<real>(0), link.mass);
    link.I_j = inv_transform(I0, ttransform<real>(inverse(link.local_link_pose)));
    return link;
}

void ArticulatedBodySpec::build(bool use_bullet, btCollisionWorld* bt_col_world) {
    int num_joints = get_num_joints();
    joint_pos_dofs.resize(num_joints);
    joint_pos_dof_starts.resize(num_joints + 1);
    joint_vel_dofs.resize(num_joints);
    joint_vel_dof_starts.resize(num_joints + 1);
    parents.resize(num_joints);

    for (int i = 0; i < num_joints; i++) {
        auto& link = links[i];
        auto& joint = joints[i];
        // Find the unit screw axis of the current joint
        joint_pos_dofs[i] = joint.pos_dof();
        joint_vel_dofs[i] = joint.vel_dof();
    }
    joint_pos_dof_starts[0] = 0;
    for (int i = 1; i <= num_joints; i++) {
        joint_pos_dof_starts[i] = joint_pos_dof_starts[i-1] + joint_pos_dofs[i-1];
    }
    num_pos_dofs = joint_pos_dof_starts[num_joints];
    joint_vel_dof_starts[0] = 0;
    for (int i = 1; i <= num_joints; i++) {
        joint_vel_dof_starts[i] = joint_vel_dof_starts[i-1] + joint_vel_dofs[i-1];
    }
    num_vel_dofs = joint_vel_dof_starts[num_joints];

    for (int i = 0; i < num_joints; i++) {
        parents[i] = links[i].parent_idx;
    }
    children_buffer.reserve(links.size()-1);
    children_buffer_starts.resize(links.size()+1);
    for (int i = 0; i < links.size(); i++) {
        children_buffer_starts[i] = children_buffer.size();
        for (int j = 1; j < links.size(); j++) {
            if (i == parents[j]) {
                children_buffer.push_back(j);
            }
        }
    }
    children_buffer_starts[links.size()] = children_buffer.size();
    std::queue<uint32_t> queue;
    queue.push(0);
    while (!queue.empty()) {
        uint32_t i = queue.front();
        queue.pop();
        uint32_t num_children = get_num_children(i);
        bfs_iteration_order.push_back(i);

        const int* i_children = get_children(i);
        for (int c = 0; c < num_children; c++) {
            queue.push(i_children[c]);
        }
    }

    build_finished = true;
}

void ArticulatedBody::init(artsim::ArticulatedBodySpec art_spec) {
    this->spec = std::move(art_spec);

    if (!spec.build_finished) {
        fprintf(stderr, "ArticulatedBodySpec not built! Call build() before creating articulation\n");
        exit(EXIT_FAILURE);
    }

    int num_pos_dofs = get_num_pos_dofs();
    int num_vel_dofs = get_num_vel_dofs();
    int num_links = get_num_links();
    int num_joints = get_num_joints();
    q.resize(num_pos_dofs, 0);
    u.resize(num_vel_dofs, 0);
    udot.resize(num_vel_dofs, 0);
    tau.resize(num_vel_dofs, 0);
    f_ext.resize(num_links, glmx::tscrew<real>(glmx::IDENTITY));
    T_link_globals.resize(num_links, glmx::ttransform<real>(glmx::IDENTITY));
    T_joint_globals.resize(num_joints, glmx::ttransform<real>(glmx::IDENTITY));

    reset();
}

void ArticulatedBody::init(Id<ArticulatedBody> art_id, ArticulatedBodySpec art_spec, Id<Material> mat_id,
                           btCollisionWorld* bt_collision_world)
{
    init(art_spec);

    this->mat_id = mat_id;

    int num_links = get_num_links();
    bt_collision_objects.resize(num_links);

    for (int i = 0; i < num_links; i++) {
        // TODO: Allocate these from a separate array!
        // TODO: Set body_id with current articulation id
        auto col_shape = spec.links[i].col_shape;
        if (col_shape.type != CollisionShape::Type::Mesh) {
            BodyId body_id = BodyId::from_articulation_link(art_id, i);
            btCollisionObject* col_obj = new btCollisionObject;
            col_obj->setCollisionShape(spec.links[i].col_shape.bt_shape);
            col_obj->setUserIndex(body_id.index);
            col_obj->setUserIndex2(body_id.generation);
            bt_collision_world->addCollisionObject(col_obj, 0b1000000, ~0b1000000);
            bt_collision_objects[i] = col_obj;
        }
    }
}

void ArticulatedBody::release() {
    for (auto& bt_col : bt_collision_objects) {
        delete bt_col;
    }
}

void ArticulatedBody::reset() {
    real* qp = q.data();
    int num_joints = get_num_joints();
    for (int i = 0; i < num_joints; i++) {
        switch (spec.joints[i].type) {
            JOINT_DOF_1_CASE {
                qp[0] = 0;
            } break;
            case JOINT_TYPE_FLOATING: {
                qp[0] = 0; qp[1] = 0; qp[2] = 0;
                qp[3] = 0; qp[4] = 0; qp[5] = 0; qp[6] = 1;
            } break;
            case JOINT_TYPE_SPHERICAL: {
                qp[0] = 0; qp[1] = 0; qp[2] = 0; qp[3] = 1;
            } break;
        }
        qp += spec.joint_pos_dofs[i];
    }
    std::fill(u.begin(), u.end(), 0);
    std::fill(udot.begin(), udot.end(), 0);
    std::fill(tau.begin(), tau.end(), 0);
    std::fill(f_ext.begin(), f_ext.end(), glmx::rscrew(glmx::IDENTITY));

    forward_kinematics();
}

void ArticulatedBody::randomize_positions() {
    thread_local std::default_random_engine engine(0);

    int num_joints = get_num_joints();
    const real pi = glm::pi<real>();
    real* qp = q.data();
    for (int i = 0; i < num_joints; i++) {
        switch (spec.joints[i].type) {
            JOINT_DOF_1_CASE {
                qp[0] = std::uniform_real_distribution<real>(-0.2*pi, 0.2*pi)(engine);
            } break;
            case JOINT_TYPE_SPHERICAL: {
                real len = std::uniform_real_distribution<real>(-0.2*pi, 0.2*pi)(engine);
                glm::tvec3<real> dir = glm::tvec3<real>(
                        std::uniform_real_distribution<real>(-1, 1)(engine),
                        std::uniform_real_distribution<real>(-1, 1)(engine),
                        std::uniform_real_distribution<real>(-1, 1)(engine)
                );
                glm::tquat<real> vexp = artsim::exp(len * normalize(dir));
                qp[0] = vexp[0]; qp[1] = vexp[1]; qp[2] = vexp[2]; qp[3] = vexp[3];
            } break;
            case JOINT_TYPE_FLOATING: {
                qp[0] = std::uniform_real_distribution<real>(-0.1, 0.1)(engine);
                qp[1] = std::uniform_real_distribution<real>(num_joints, num_joints+1)(engine);
                qp[2] = std::uniform_real_distribution<real>(-0.1, 0.1)(engine);

                real len = std::uniform_real_distribution<real>(-0.2f*pi, 0.2f*pi)(engine);
                glm::tvec3<real> dir = glm::tvec3<real>(
                        std::uniform_real_distribution<real>(-1, 1)(engine),
                        std::uniform_real_distribution<real>(-1, 1)(engine),
                        std::uniform_real_distribution<real>(-1, 1)(engine)
                );
                glm::tquat<real> vexp = artsim::exp(len * normalize(dir));
                qp[3] = vexp[0]; qp[4] = vexp[1]; qp[5] = vexp[2]; qp[6] = vexp[3];
            } break;
        }
        qp += spec.joint_pos_dofs[i];
    }

    forward_kinematics();
}

void ArticulatedBody::forward_kinematics() {
    artsim::calc_transforms(spec, q.data(), T_joint_globals.data(), T_link_globals.data());
}

void ArticulatedBody::update_colliders() {
    int num_joints = get_num_joints();
    for (int i = 0; i < num_joints; i++) {
        bt_collision_objects[i]->setWorldTransform(btconv(T_link_globals[i]));
    }
}

void ArticulatedBody::forward_dynamics(const glm::rvec3& gravity, real dt) {
    artsim::featherstone_forward_dynamics(spec, gravity, dt, f_ext.data(), q.data(), u.data(), tau.data(),
                                          OUT udot.data());
}

void ArticulatedBody::integrate(real dt) {
    artsim::integrate_implicit_euler(spec, dt, udot.data(), INOUT q.data(), INOUT u.data());
    forward_kinematics();
}

void ArticulatedBody::simulate(const glm::rvec3& gravity, real dt) {
    forward_dynamics(gravity, dt);
    integrate(dt);
}

void ArticulatedBody::mass_matrix(
        OUT glmx::dynmat_view<real> M, real dt) {
    artsim::mass_matrix(spec, dt, q.data(), OUT M);
}

void ArticulatedBody::multiply_inverse_mass_matrix(
        glmx::dynmat_view<real> X, OUT glmx::dynmat_view<real> Minv_X, real dt) {
    artsim::multiply_inverse_mass_matrix(spec, dt, q.data(), X, OUT Minv_X);
}

real ArticulatedBody::get_joint_pos_1dof(int joint_idx) const {
    assert(spec.joint_pos_dofs[joint_idx] == 1);
    uint32_t jidx_start = spec.joint_pos_dof_starts[joint_idx];
    return q[jidx_start];
}

glm::tquat<real> ArticulatedBody::get_joint_pos_spherical(int joint_idx) const {
    assert(spec.joint_pos_dofs[joint_idx] == 4);
    uint32_t jidx_start = spec.joint_pos_dof_starts[joint_idx];
    return glm::make_quat(q.data() + jidx_start);
}

glmx::ttransform<real> ArticulatedBody::get_root_transform() const {
    assert(spec.floating);
    return {glm::make_vec3(q.data()), glm::mat3_cast(glm::make_quat(q.data() + 3))};
}

real ArticulatedBody::get_joint_vel_1dof(int joint_idx) const {
    assert(spec.joint_vel_dofs[joint_idx] == 1);
    uint32_t jidx_start = spec.joint_vel_dof_starts[joint_idx];
    return u[jidx_start];
}

glm::rvec3 ArticulatedBody::get_joint_vel_spherical(int joint_idx) const {
    assert(spec.joint_vel_dofs[joint_idx] == 3);
    uint32_t jidx_start = spec.joint_vel_dof_starts[joint_idx];
    return glm::make_vec3(u.data() + jidx_start);
}

glmx::rscrew ArticulatedBody::get_root_vel() const {
    assert(spec.floating);
    return glmx::make_tscrew(u.data());
}

void ArticulatedBody::set_joint_pos_1dof(int joint_idx, real qj) {
    assert(spec.joint_pos_dofs[joint_idx] == 1);
    uint32_t jidx_start = spec.joint_pos_dof_starts[joint_idx];
    q[jidx_start] = qj;
}

void ArticulatedBody::set_joint_pos_spherical(int joint_idx, const glm::tquat<real>& qj) {
    assert(spec.joint_pos_dofs[joint_idx] == 4);
    uint32_t jidx_start = spec.joint_pos_dof_starts[joint_idx];
    q[jidx_start+0] = qj[0];
    q[jidx_start+1] = qj[1];
    q[jidx_start+2] = qj[2];
    q[jidx_start+3] = qj[3];
}

void ArticulatedBody::set_root_transform(const ttransform<real>& rootT) {
    assert(spec.floating);
    glm::quat rot = glm::quat_cast(rootT.R);
    q[0] = rootT.v[0];
    q[1] = rootT.v[1];
    q[2] = rootT.v[2];
    q[3] = rot[0];
    q[4] = rot[1];
    q[5] = rot[2];
    q[6] = rot[3];
}

void ArticulatedBody::set_joint_vel_1dof(int joint_idx, real qj) {
    assert(spec.joint_vel_dofs[joint_idx] == 1);
    uint32_t jidx_start = spec.joint_vel_dof_starts[joint_idx];
    q[jidx_start] = qj;
}
void ArticulatedBody::set_joint_vel_spherical(int joint_idx, const glm::rvec3& qj) {
    assert(spec.joint_vel_dofs[joint_idx] == 3);
    uint32_t jidx_start = spec.joint_vel_dof_starts[joint_idx];
    q[jidx_start+0] = qj[0];
    q[jidx_start+1] = qj[1];
    q[jidx_start+2] = qj[2];
}
void ArticulatedBody::set_root_vel(const glmx::rscrew& V) {
    assert(spec.floating);
    q[0] = V[0];
    q[1] = V[1];
    q[2] = V[2];
    q[3] = V[3];
    q[4] = V[4];
    q[5] = V[5];
}

glmx::rtransform ArticulatedBody::get_global_joint_trans(int joint_idx) const {
    return T_joint_globals[joint_idx];
}

glmx::rtransform ArticulatedBody::get_global_link_trans(int link_idx) const {
    return T_link_globals[link_idx];
}

void World::init(WorldConfig world_cfg) {
    cfg = std::move(world_cfg);
    auto bt_collision_config = new btDefaultCollisionConfiguration;
    auto bt_dispatcher = new btCollisionDispatcher(bt_collision_config);
    auto bt_broadphase = new btDbvtBroadphase;
    this->bt_collision_world = new btCollisionWorld(bt_dispatcher, bt_broadphase, bt_collision_config);

    if (cfg.create_plane) {
        bt_plane_col = new btCollisionObject;
        bt_plane_col->setCollisionShape(new btStaticPlaneShape(btVector3(0, 1, 0), 0));
        bt_plane_col->setWorldTransform(btTransform::getIdentity());
        bt_plane_col->setUserIndex(0);
        bt_plane_col->setUserIndex2(0);
        this->bt_collision_world->addCollisionObject(bt_plane_col, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
    }
}

void World::simulate(real dt) {
    for (auto& art : articulated_bodies) {
        art.forward_kinematics();
        art.update_colliders();
    }
    bt_collision_world->performDiscreteCollisionDetection();
    solve_contacts();
}

void World::solve_contacts() {
    contact_points.clear();

    art_ground_contacts.clear();
    art_ground_contacts.resize(articulated_bodies.size());
    art_ground_contact_forces.clear();
    art_ground_contact_forces.resize(articulated_bodies.size());

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
            Id<ContactPoint> cp_id = contact_points.make();
            auto cp = contact_points.get(cp_id);
            cp->bt_manifold = manifold;
            cp->pos = glmconv(pt.getPositionWorldOnB());
            cp->normal = glmconv(pt.m_normalWorldOnB);
            cp->depth = -pt.getDistance();
            cp->area = 0;
            cp->body1_id = body1_id;
            cp->body2_id = body2_id;
            if (body1_id.is_articulation() && body2_id.is_ground()) {
                auto [art_id, link_idx] = body1_id.get_articulation_id();
                int art_idx = articulated_bodies.get_item_idx(art_id);
                auto& art_contacts = art_ground_contacts[art_idx];
                art_contacts.push_back(*cp);
            }
        }
    }
    int art_idx = 0;
    for (auto& art : articulated_bodies) {
        auto& art_contacts = art_ground_contacts[art_idx];
        auto& art_contact_forces = art_ground_contact_forces[art_idx];
        art_contact_forces.resize(art_contacts.size());
        artsim::euler_step_with_collision(
                cfg.contact_solver_type, cfg.max_iters,
                art.get_spec(), *material_db.get_material(art.get_mat_id()), cfg.gravity, cfg.dt,
                art.get_external_force_buf(), art.get_internal_force_buf(),
                art_contacts.data(), art_contacts.size(),
                INOUT art.get_pos_buf(), INOUT art.get_vel_buf(), OUT art.get_acc_buf(),
                OUT art_contact_forces.data());
        art_idx++;
    }
}
