//
// Created by lasagnaphil on 8/21/21.
//

#include <artsim/math/bullet.h>
#include <artsim/art_body.h>
#include <artsim/art_dynamics.h>
#include <artsim/artsim.h>
#include <artsim/world.h>

#include <random>

using namespace glmx;

namespace artsim {

void ArticulatedBodySpec::add_link_and_joint(Link link, Joint joint, const std::string& name) {
    if (joint.type == JOINT_TYPE_FLOATING) {
        if (!links.empty() || !joints.empty()) {
            fprintf(stderr, "Error in ArticulatedBody::add_link_and_joint: "
                            "Free joint can only be added at the root!\n");
            return;
        }
        floating = true;
    }
    links.push_back(std::move(link));
    joints.push_back(std::move(joint));
    names.push_back(name);
}

void ArticulatedBodySpec::build() {

    build_finished = true;
}

int ArticulatedBodySpec::get_index(const char* name) const {
    int i;
    for (i = 0; i < names.size(); i++) {
        if (names[i] == name) break;
    }
    if (i == names.size()) return -1;
    else return i;
}

/*
void ArticulatedBodySpec::scale_link(int link_idx, const rvec3& scale, bool scale_shapes) {
    auto& link = links[link_idx];
    if (scale_shapes) {
        link.col_shape.scale *= scale;
    }
    link.local_link_pose.v = glm::rvec3(scale) * link.local_link_pose.v;
    uint32_t num_children = get_num_children(link_idx);
    const int* children = get_children(link_idx);
    for (int i = 0; i < num_children; i++) {
        uint32_t child_idx = children[i];
        links[child_idx].local_joint_pose.v =
                glm::rvec3(scale) * links[child_idx].local_joint_pose.v;
    }
}

void ArticulatedBodySpec::scale_link(int link_idx, const rmat3& rot, const rvec3& scale) {
    auto& link = links[link_idx];
    auto T = rot * glm::rmat3(scale[0], 0, 0, 0, scale[1], 0, 0, 0, scale[2]) * glm::transpose(rot);
    link.local_link_pose.v = T * link.local_link_pose.v;
    uint32_t num_children = get_num_children(link_idx);
    const int* children = get_children(link_idx);
    for (int i = 0; i < num_children; i++) {
        uint32_t child_idx = children[i];
        links[child_idx].local_joint_pose.v =
                T * links[child_idx].local_joint_pose.v;
    }
}
 */

PoseTree ArticulatedBodySpec::get_pose_tree() {
    PoseTree poseTree;
    int num_links = get_num_links();
    poseTree.allNodes.resize(num_links);
    for (int i = 0; i < num_links; i++) {
        auto& node = poseTree.allNodes[i];
        node.name = names[i];
        node.parent = parents[i];
        node.childJoints.resize(get_num_children(i));
        auto children_data = get_children(i);
        for (int k = 0; k < node.childJoints.size(); k++) {
            node.childJoints[k] = children_data[k];
        }
        node.offset = links[i].local_joint_pose.v;
    }
    return poseTree;
}

void ArticulatedBody::init(World* _world, Id<ArticulatedBodySpec> art_spec_id) {
    world = _world;
    spec_id = art_spec_id;
    auto& spec = *world->get_art_body_spec(art_spec_id);
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
    q_target.resize(num_pos_dofs, 0);

    reset();
}

void ArticulatedBody::reset() {
    auto& spec = *world->get_art_body_spec(spec_id);
    int num_joints = get_num_joints();
    real* qp = q.data();
    if (spec.initial_state.empty()) {
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
    }
    else {
        q = spec.initial_state;
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

    forward_kinematics();
}

void ArticulatedBody::randomize_positions() {
    thread_local std::default_random_engine engine(std::time(nullptr));
    auto& spec = *world->get_art_body_spec(spec_id);

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
                glm::tquat<real> vexp = glmx::exp(len * normalize(dir));
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
                glm::tquat<real> vexp = glmx::exp(len * normalize(dir));
                qp[3] = vexp[0]; qp[4] = vexp[1]; qp[5] = vexp[2]; qp[6] = vexp[3];
            } break;
        }
        qp += spec.joint_pos_dofs[i];
    }

    forward_kinematics();
}

void ArticulatedBody::forward_kinematics() {
    auto& spec = *world->get_art_body_spec(spec_id);

    for (uint32_t i : spec.bfs_iteration_order) {
        int cur_pos_dof = spec.joint_pos_dof_starts[i];
        int cur_vel_dof = spec.joint_vel_dof_starts[i];
        auto& joint = spec.joints[i];
        auto& link = spec.links[i];
        RigidBody* body = world->get_rigid_body(bodies[i]);

        RigidBody* parent_body;
        rquat_transform parent_world_trans;
        rscrew parent_body_vel;
        if (i == 0) {
            parent_body = nullptr;
            parent_world_trans = rquat_transform(IDENTITY);
            parent_body_vel = rscrew(IDENTITY);
        }
        else {
            parent_body = world->get_rigid_body(bodies[spec.parents[i]]);
            parent_world_trans = parent_body->world_trans;
            parent_body_vel = parent_body->body_vel;
        }

        rquat_transform local_trans;

        switch (joint.type) {
            JOINT_DOF_1_CASE {
                rscrew S = get_joint_screw(joint.type);
                local_trans = body->offset_from_com * move(S, q[cur_pos_dof]);
                body->world_trans = parent_world_trans * local_trans;
            } break;
            case JOINT_TYPE_SPHERICAL: {
                glm::rquat q_j = glm::make_quat<real>(q.data() + cur_pos_dof);
                body->world_trans = parent_world_trans * body->offset_from_com * q_j;
            } break;
            case JOINT_TYPE_FLOATING: {
                glm::rvec3 v_j = glm::make_vec3<real>(q.data() + cur_pos_dof);
                glm::rquat q_j = glm::make_quat<real>(q.data() + cur_pos_dof + 3);
                body->world_trans = parent_world_trans * rquat_transform(v_j, q_j);
            } break;
        }

        rscrew v0 = calc_v0(spec.joints[i], u.data() + cur_vel_dof);
        body->body_vel = Ad(glmx::inverse(local_trans), parent_body_vel) + v0;
        // body->body_vel = Ad_inv(local_trans, parent_body_vel) + v0;
    }
}

void ArticulatedBody::update_colliders() {
    /* TODO
    int num_joints = get_num_joints();
    for (int i = 0; i < num_joints; i++) {
        bt_collision_objects[i]->setWorldTransform(btconv(global_link_trans[i]));
    }
     */
}

void ArticulatedBody::forward_dynamics(const glm::rvec3& gravity, real dt) {
    auto& spec = *world->get_art_body_spec(spec_id);
    std::vector<rscrew> f_ext(bodies.size());
    for (int i = 0; i < bodies.size(); i++) {
        f_ext[i] = world->get_rigid_body(bodies[i])->body_f_ext;
    }
    artsim::featherstone_forward_dynamics(spec, gravity, dt, f_ext.data(), q.data(), u.data(), tau.data(), q_target.data(),
                                          OUT udot.data());
}

void ArticulatedBody::forward_dynamics_with_contact(const rvec3& gravity, real dt) {
    auto& spec = *world->get_art_body_spec(spec_id);
    std::vector<rscrew> f_ext(bodies.size()), f_c(bodies.size());
    for (int i = 0; i < bodies.size(); i++) {
        f_ext[i] = world->get_rigid_body(bodies[i])->body_f_ext;
        f_c[i] = world->get_rigid_body(bodies[i])->body_f_c;
    }
    artsim::featherstone_forward_dynamics(spec, gravity, dt, f_ext.data(), f_c.data(), q.data(), u.data(), tau.data(), q_target.data(),
                                          OUT udot.data());
}

void ArticulatedBody::integrate(real dt) {
    auto& spec = *world->get_art_body_spec(spec_id);
    artsim::integrate_implicit_euler(spec, dt, udot.data(), q.data(), u.data());
    forward_kinematics();
}

void ArticulatedBody::simulate(const glm::rvec3& gravity, real dt) {
    forward_dynamics(gravity, dt);
    integrate(dt);
    forward_kinematics();
}

void ArticulatedBody::mass_matrix(
        OUT glmx::dynmat_view<real> M, real dt) {
    auto& spec = *world->get_art_body_spec(spec_id);
    artsim::mass_matrix(spec, dt, q.data(), OUT M);
}

void ArticulatedBody::multiply_inverse_mass_matrix(
        glmx::dynmat_view<real> X, OUT glmx::dynmat_view<real> Minv_X, real dt) {
    auto& spec = *world->get_art_body_spec(spec_id);
    artsim::multiply_inverse_mass_matrix(spec, dt, q.data(), X, OUT Minv_X);
}

real ArticulatedBody::get_joint_pos_1dof(int joint_idx) const {
    auto& spec = *world->get_art_body_spec(spec_id);
    assert(spec.joint_pos_dofs[joint_idx] == 1);
    uint32_t jidx_start = spec.joint_pos_dof_starts[joint_idx];
    return q[jidx_start];
}

glm::tquat<real> ArticulatedBody::get_joint_pos_spherical(int joint_idx) const {
    auto& spec = *world->get_art_body_spec(spec_id);
    assert(spec.joint_pos_dofs[joint_idx] == 4);
    uint32_t jidx_start = spec.joint_pos_dof_starts[joint_idx];
    return glm::make_quat(q.data() + jidx_start);
}

glmx::ttransform<real> ArticulatedBody::get_root_transform() const {
    auto& spec = *world->get_art_body_spec(spec_id);
    assert(spec.floating);
    return {glm::make_vec3(q.data()), glm::mat3_cast(glm::make_quat(q.data() + 3))};
}

real ArticulatedBody::get_joint_vel_1dof(int joint_idx) const {
    auto& spec = *world->get_art_body_spec(spec_id);
    assert(spec.joint_vel_dofs[joint_idx] == 1);
    uint32_t jidx_start = spec.joint_vel_dof_starts[joint_idx];
    return u[jidx_start];
}

glm::rvec3 ArticulatedBody::get_joint_vel_spherical(int joint_idx) const {
    auto& spec = *world->get_art_body_spec(spec_id);
    assert(spec.joint_vel_dofs[joint_idx] == 3);
    uint32_t jidx_start = spec.joint_vel_dof_starts[joint_idx];
    return glm::make_vec3(u.data() + jidx_start);
}

glmx::rscrew ArticulatedBody::get_root_vel() const {
    auto& spec = *world->get_art_body_spec(spec_id);
    assert(spec.floating);
    return glmx::make_tscrew(u.data());
}

void ArticulatedBody::set_joint_pos_1dof(int joint_idx, real qj) {
    auto& spec = *world->get_art_body_spec(spec_id);
    assert(spec.joint_pos_dofs[joint_idx] == 1);
    uint32_t jidx_start = spec.joint_pos_dof_starts[joint_idx];
    q[jidx_start] = qj;
}

void ArticulatedBody::set_joint_pos_spherical(int joint_idx, const glm::tquat<real>& qj) {
    auto& spec = *world->get_art_body_spec(spec_id);
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
    auto& spec = *world->get_art_body_spec(spec_id);
    assert(spec.joint_vel_dofs[joint_idx] == 1);
    uint32_t jidx_start = spec.joint_vel_dof_starts[joint_idx];
    q[jidx_start] = qj;
}
void ArticulatedBody::set_joint_vel_spherical(int joint_idx, const glm::rvec3& qj) {
    auto& spec = *world->get_art_body_spec(spec_id);
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

glm::rvec3 ArticulatedBody::get_center_of_mass() const {
    auto& spec = *world->get_art_body_spec(spec_id);
    glm::rvec3 com(0, 0, 0);
    real total_mass = 0;
    int num_links = spec.get_num_links();
    for (int lidx = 0; lidx < num_links; lidx++) {
        auto& rb = *world->get_rigid_body(bodies[lidx]);
        auto body_com = glm::conjugate(rb.offset_from_com.q) * rb.world_trans.v - rb.offset_from_com.v;
        com += spec.links[lidx].mass * body_com;
        total_mass += spec.links[lidx].mass;
    }
    com /= total_mass;
    return com;
}

}
