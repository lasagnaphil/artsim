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

#include <Eigen/Dense>

#include <Tracy.hpp>

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
            // TODO: Calculate proper inertia (Resources: https://abhilashreddy.com/writing/6/mesh_props.html)
            return tsmat3x3<real>(1, 1, 1, 0, 0, 0);
        }
        default: return tsmat3x3<real>(0, 0, 0, 0, 0, 0);
    }
}

CollisionShape CollisionShape::make_ground() {
    CollisionShape shape;
    shape.type = CollisionShape::Type::Ground;
    shape.scale = glm::rvec3(1);
    shape.bt_shape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
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

void RigidBody::init(RigidBodySpec rb_spec) {
    this->spec = std::move(rb_spec);
    pos = glm::rvec3(0);
    vel = glm::rvec3(0);
    rot = glm::identity<glm::rquat>();
    angvel = glm::rvec3(0);
}

void RigidBody::init(Id<RigidBody> rb_id, RigidBodySpec rb_spec, Id<Material> mat_id,
                     btCollisionWorld* bt_collision_world) {
    init(rb_spec);
    this->mat_id = mat_id;
    auto col_shape = spec.col_shape;
    if (col_shape.type != CollisionShape::Type::Mesh) {
        BodyId body_id = BodyId::from_rigid_body(rb_id);
        bt_collision_object = new btCollisionObject;
        bt_collision_object->setCollisionShape(spec.col_shape.bt_shape);
        bt_collision_object->setWorldTransform(btTransform::getIdentity());
        bt_collision_object->setUserIndex(body_id.index);
        bt_collision_object->setUserIndex2(body_id.generation);
        if (col_shape.type == CollisionShape::Type::Ground) {
            bt_collision_world->addCollisionObject(bt_collision_object, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
        }
        else {
            bt_collision_world->addCollisionObject(bt_collision_object, 0b1000000, ~0b1000000);
        }
    }
}

void RigidBody::release(btCollisionWorld* bt_world) {
    bt_world->removeCollisionObject(bt_collision_object);
    delete bt_collision_object;
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

void ArticulatedBodySpec::build() {
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

void ArticulatedBody::release(btCollisionWorld* bt_world) {
    for (btCollisionObject* bt_col : bt_collision_objects) {
        bt_world->removeCollisionObject(bt_col);
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
    forward_kinematics();
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
}

void World::simulate(real dt) {
    for (auto& art : articulated_bodies) {
        art.forward_kinematics();
        art.update_colliders();
    }
    bt_collision_world->performDiscreteCollisionDetection();
    integrate_with_contacts();
}

namespace std {
template<>
struct hash<BodyId> {
    std::size_t operator()(const BodyId& id) const {
        using std::hash;
        return hash<uint32_t>()(id.index) ^ (hash<uint32_t>()(id.generation) << 1);
    }
};
}

Eigen::Matrix<real, 3, 1> glm_to_eigen(const glm::rvec3& v) {
    return Eigen::Map<Eigen::Matrix<real, 3, 1>>((real*)&v[0]);
}

void World::integrate_with_contacts() {
    ZoneScoped

    contact_points.clear();

    std::vector<Id<ContactPoint>> contact_point_ids;

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
                Id<ContactPoint> cp_id = contact_points.make();
                contact_point_ids.push_back(cp_id);
                auto cp = contact_points.get(cp_id);
                cp->bt_manifold = manifold;
                cp->pos = glmconv(pt.getPositionWorldOnB());
                cp->normal = glmconv(pt.m_normalWorldOnB);
                cp->depth = -pt.getDistance();
                cp->area = 0;
                cp->body1_id = body1_id;
                cp->body2_id = body2_id;
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

    std::unordered_map<Id<ArticulatedBody>, std::vector<int>> art_contact_points_map;
    std::unordered_map<Id<RigidBody>, std::vector<int>> rb_contact_points_map;

    auto insert_contact_info = [&](BodyId bid, int cidx) {
        if (bid.is_articulation()) {
            auto [art_id, art_lidx] = bid.get_articulation_id();
            auto it = art_contact_points_map.find(art_id);
            if (it == art_contact_points_map.end()) {
                art_contact_points_map.insert({art_id, {cidx}});
            }
            else {
                it->second.push_back(cidx);
            }
        }
        else {
            auto rb_id = bid.get_rigid_body_id();
            auto it = rb_contact_points_map.find(rb_id);
            if (it == rb_contact_points_map.end()) {
                rb_contact_points_map.insert({rb_id, {cidx}});
            }
            else {
                it->second.push_back(cidx);
            }
        }
    };

    {
        ZoneNamedN(InsertContactInfo, "InsertContactInfo", true);

        for (int cidx = 0; cidx < contact_point_ids.size(); cidx++) {
            Id<ContactPoint> cp_id = contact_point_ids[cidx];
            auto* cp = contact_points.get(cp_id);
            insert_contact_info(cp->body1_id, cidx);
            insert_contact_info(cp->body2_id, cidx);
        }
    }

    std::vector<MatrixXr> art_contact_jacobian_map(art_contact_points_map.size());
    std::vector<MatrixXr> rb_contact_jacobian_map(rb_contact_points_map.size());
    std::vector<rvec3> c(num_contacts);
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

            VectorXr art1_udot_bar(art1_num_vel_dofs);

            featherstone_forward_dynamics(art1_spec, cfg.gravity, cfg.dt,
                                          art1_f_ext, art1_q, art1_u, art1_tau, OUT art1_udot_bar.data());

            VectorXr art1_u_bar = Eigen::Map<VectorXr>(art1_u, art1_num_vel_dofs) + art1_udot_bar * cfg.dt;

            art_contact_jacobian_map[i] = MatrixXr(art1_num_vel_dofs, 3*art1_num_contact_points);
            auto& Jc_T = art_contact_jacobian_map[i];

            for (int c = 0; c < cidx_list.size(); c++) {
                int cidx = cidx_list[c];
                Id<ContactPoint> cp_id = contact_point_ids[cidx];
                ContactPoint* cp = contact_points.get(cp_id);
                bool body1_is_art1 = cp->body1_id.is_articulation() && cp->body1_id.get_articulation_id().first == art1_id;
                BodyId body1_id = body1_is_art1? cp->body1_id : cp->body2_id;
                BodyId body2_id = body1_is_art1? cp->body2_id : cp->body1_id;
                auto [_, art1_lidx] = body1_id.get_articulation_id();
                // auto tangent_u = Ez<real>();
                // auto tangent_v = glm::cross(cp->normal, tangent_u);
                // auto contact_T = ttransform<real>(cp->pos, glm::tmat3x3<real>(tangent_u, tangent_v, cp->normal));
                auto contact_T = rtransform(cp->pos, mat3_cast(rotation(Ez<real>(), cp->normal)));
                auto contact_rel_T = contact_T / art1.get_global_joint_trans(art1_lidx);
                rtransform* T_joint_global = art1.get_global_joint_trans_buf();
                dynmat_view<real> Jc_T_view(Jc_T.data() + 3*c*art1_num_vel_dofs, art1_num_vel_dofs, 3);
                calc_linear_jacobian_transpose(art1.get_spec(), art1_lidx, contact_rel_T, T_joint_global,
                                               OUT Jc_T_view);
            }

            VectorXr tau_star = Jc_T.transpose() * art1_u_bar;

            const real beta = 0.01;
            const real slop = 5e-5;

            for (int cidx : cidx_list) {
                Id<ContactPoint> cp_id = contact_point_ids[cidx];
                ContactPoint* cp = contact_points.get(cp_id);
                bool body1_is_art1 = cp->body1_id.is_articulation() && cp->body1_id.get_articulation_id().first == art1_id;
                BodyId body1_id = body1_is_art1? cp->body1_id : cp->body2_id;
                BodyId body2_id = body1_is_art1? cp->body2_id : cp->body1_id;
                glm::rvec3 tau = make_vec3<real>(tau_star.data() + 3*cidx);
                tau.z -= beta / cfg.dt * glm::max<real>(cp->depth - slop, 0);
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
                        M_delassus(cidx_list[i], cidx_list[k]) = glm::make_mat3(M_contact_inv_eigen.data());
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
        iterative_contact_solver(cfg.contact_solver_type, cfg.max_iters, mat.data(), cfg.dt, num_contacts,
                                 M_delassus, OUT c.data(), OUT lambda.data());
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1);
    printf("Contact solver: %lld ns\n", duration.count());

    {
        ZoneNamedN(IntegrateWithContacts, "IntegrateWithContacts", true);

        for (auto& [art_id, cidx_list] : art_contact_points_map) {
            auto& art = *articulated_bodies.get(art_id);
            int num_joints = art.get_num_joints();
            std::vector<rscrew> f_ext_tot(num_joints);
            std::copy_n(art.get_external_force_buf(), num_joints, f_ext_tot.data());
            for (int i = 0; i < cidx_list.size(); i++) {
                int cidx = cidx_list[i];
                auto cp = contact_points.get(contact_point_ids[cidx]);
                bool body1_is_art1 = cp->body1_id.is_articulation() && cp->body1_id.get_articulation_id().first == art_id;
                BodyId body1_id = body1_is_art1? cp->body1_id : cp->body2_id;
                BodyId body2_id = body1_is_art1? cp->body2_id : cp->body1_id;
                auto [_, art_lidx] = body1_id.get_articulation_id();
                auto contact_T = rtransform(cp->pos, mat3_cast(rotation(Ez<real>(), cp->normal)));
                auto contact_rel_T = art.get_global_joint_trans(art_lidx) / contact_T;
                f_ext_tot[art_lidx] += AdT(contact_rel_T, rscrew(rvec3(0), lambda[cidx] / cfg.dt));
            }
            auto& spec = art.get_spec();
            real* q = art.get_pos_buf(); real* u = art.get_vel_buf(); real* udot = art.get_acc_buf();
            featherstone_forward_dynamics(spec, cfg.gravity, cfg.dt,
                                          f_ext_tot.data(), q, u,
                                          art.get_internal_force_buf(), OUT udot);
            integrate_implicit_euler(spec, cfg.dt, udot, INOUT q, INOUT u);
        }

        for (auto& [rb1_id, cidx_list] : rb_contact_points_map) {
            // TODO
        }

    }
}
