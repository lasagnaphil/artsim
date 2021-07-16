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

#include <Discregrid/All>

#include <queue>
#include <random>

#include <fmt/core.h>

#include <Eigen/Dense>

#include <Tracy.hpp>

static glm::rvec3 eigen_to_glm(const Eigen::Vector3d& v) {
    return {v(0), v(1), v(2)};
}

static Eigen::Vector3i glm_to_eigen(const glm::ivec3& v) {
    return Eigen::Vector3i(v[0], v[1], v[2]);
}

using namespace artsim;
using namespace glmx;

void CollisionMesh::init_from_obj(const char *filename, real sdf_grid_size) {
    fmt::print("Loading mesh {}\n", filename);
    objfile.load_obj(filename);
    Discregrid::TriangleMesh mesh(filename);
    fmt::print("Done\n");

    fmt::print("Setting up SDF grid...\n");
    Discregrid::MeshDistance md(mesh);

    Eigen::AlignedBox3d domain;
    domain.setEmpty();
    for (auto const& x : mesh.vertices())
    {
        domain.extend(x);
    }
    domain.max() += 1.0e-3 * domain.diagonal().norm() * Eigen::Vector3d::Ones();
    domain.min() -= 1.0e-3 * domain.diagonal().norm() * Eigen::Vector3d::Ones();

    fmt::print("Done\n");

    Eigen::Vector3d size_cm = (domain.max() - domain.min()) / sdf_grid_size;
    glm::uvec3 size_i = glm::round(glm::dvec3(size_cm[0], size_cm[1], size_cm[2]));
    std::array<unsigned int, 3> res = {size_i[0], size_i[1], size_i[2]};

    glm::rvec3 grid_bounds = sdf_grid_size * glm::rvec3(size_i);
    Eigen::Vector3d domain_bounds = domain.max() - domain.min();
    Eigen::Vector3d domain_extra = domain_bounds - Eigen::Vector3d(grid_bounds[0], grid_bounds[1], grid_bounds[2]);
    domain_extra += 1e-6 * Eigen::Vector3d::Ones();
    domain.max() += 0.5 * domain_extra;
    domain.min() -= 0.5 * domain_extra;

    fmt::print("Generating SDF of size ({}, {}, {})...\n", res[0], res[1], res[2]);
    sdf_grid = Discregrid::CubicLagrangeDiscreteGrid(domain, res);
    auto func = [&md](Eigen::Vector3d const& xi) {return md.signedDistanceCached(xi); };
    sdf_grid.addFunction(func, true);
    fmt::print("Done\n");
}

real CollisionShape::mass(real density) {
    switch (type) {
        case Type::Box: return density * scale.x * scale.y * scale.z;
        case Type::Sphere: return real(4.0 / 3.0) * glm::pi<real>() * scale.x * scale.y * scale.z;
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

CollisionShape CollisionShape::make_mesh(Id<CollisionMesh> col_mesh, glm::rvec3 scale) {
    CollisionShape shape;
    shape.type = CollisionShape::Type::Mesh;
    shape.scale = scale;
    shape.mesh.id = col_mesh;
    shape.bt_shape = nullptr;
    return shape;
}

CollisionShape CollisionShape::make_mesh(real cell_size, glm::rvec3 scale) {
    CollisionShape shape;
    shape.type = CollisionShape::Type::Mesh;
    shape.scale = scale;
    shape.mesh.id = {};
    shape.mesh.cell_size = cell_size;
    shape.bt_shape = nullptr;
    return shape;
}

void RigidBody::init(RigidBodySpec rb_spec) {
    this->spec = rb_spec;
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
        BodyLinkId body_id = BodyLinkId::from_rigid_body(rb_id);
        bt_collision_object = new btCollisionObject;
        bt_collision_object->setCollisionShape(spec.col_shape.bt_shape);
        bt_collision_object->setWorldTransform(btTransform::getIdentity());
        bt_collision_object->setUserIndex(body_id.index);
        bt_collision_object->setUserIndex2(body_id.generation);
        if (col_shape.type == CollisionShape::Type::Ground) {
            bt_collision_world->addCollisionObject(bt_collision_object, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
        }
        else {
            bt_collision_world->addCollisionObject(bt_collision_object, 0b1000000, ~0);
        }
    }
}

void RigidBody::release(btCollisionWorld* bt_world) {
    bt_world->removeCollisionObject(bt_collision_object);
    delete bt_collision_object;
}

Link Link::create(CollisionShape col_shape, real density,
                  ttransform<real> local_joint_pose, ttransform<real> local_link_pose,
                  int parent_idx, Id<Material> mat_id, std::string obj_filename) {
    Link link;
    link.density = density;
    link.mass = col_shape.mass(density);
    link.inertia = col_shape.inertia(density);
    link.col_shape = col_shape;
    link.local_joint_pose = local_joint_pose;
    link.local_link_pose = local_link_pose;
    link.parent_idx = parent_idx;
    link.mat_id = mat_id;
    link.obj_filename = obj_filename;

    auto I0 = tspmat<real>(tsmat3x3<real>(link.inertia), glm::tvec3<real>(0), link.mass);
    link.I_j = inv_transform(I0, ttransform<real>(inverse(link.local_link_pose)));
    return link;
}

Link
Link::create(CollisionShape col_shape, real mass, glmx::tsmat3x3<real> inertia, glmx::ttransform<real> local_joint_pose,
             glmx::ttransform<real> local_link_pose, int parent_idx, Id<Material> mat_id, std::string obj_filename) {
    Link link;
    link.mass = mass;
    link.inertia = inertia;
    link.density = link.mass / col_shape.mass(1);
    link.col_shape = col_shape;
    link.local_joint_pose = local_joint_pose;
    link.local_link_pose = local_link_pose;
    link.parent_idx = parent_idx;
    link.mat_id = mat_id;
    link.obj_filename = obj_filename;

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
    f_ext.resize(num_links, glmx::rscrew(glmx::IDENTITY));
    q_target.resize(num_pos_dofs, 0);

    global_link_trans.resize(num_links, glmx::rtransform(glmx::IDENTITY));
    global_joint_trans.resize(num_joints, glmx::rtransform (glmx::IDENTITY));
    global_link_vel.resize(num_links, glmx::rscrew(glmx::IDENTITY));

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
            BodyLinkId body_id = BodyLinkId::from_articulation_link(art_id, i);
            btCollisionObject* col_obj = new btCollisionObject;
            col_obj->setCollisionShape(spec.links[i].col_shape.bt_shape);
            col_obj->setUserIndex(body_id.index);
            col_obj->setUserIndex2(body_id.generation);
            bt_collision_world->addCollisionObject(col_obj, 0b1000000, ~0b1000000);
            // bt_collision_world->addCollisionObject(col_obj, 0b1000000, ~0);
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
    int num_joints = get_num_joints();
    real* qp = q.data();
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
    qp = q_target.data();
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
    artsim::calc_transforms(spec, q.data(), OUT global_joint_trans.data(), OUT global_link_trans.data());
    artsim::calc_velocities(spec, q.data(), u.data(), OUT global_link_vel.data());
}

void ArticulatedBody::update_colliders() {
    int num_joints = get_num_joints();
    for (int i = 0; i < num_joints; i++) {
        bt_collision_objects[i]->setWorldTransform(btconv(global_link_trans[i]));
    }
}

void ArticulatedBody::forward_dynamics(const glm::rvec3& gravity, real dt) {
    artsim::featherstone_forward_dynamics(spec, gravity, dt, f_ext.data(), q.data(), u.data(), tau.data(), q_target.data(),
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
    return global_joint_trans[joint_idx];
}

glmx::rtransform ArticulatedBody::get_global_link_trans(int link_idx) const {
    return global_link_trans[link_idx];
}

glm::rvec3 ArticulatedBody::get_center_of_mass() const {
    glm::rvec3 com(0, 0, 0);
    real total_mass = 0;
    int num_links = spec.get_num_links();
    for (int lidx = 0; lidx < num_links; lidx++) {
        com += spec.links[lidx].mass * global_link_trans[lidx].v;
        total_mass += spec.links[lidx].mass;
    }
    com /= total_mass;
    return com;
}
