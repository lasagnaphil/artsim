//
// Created by lasagnaphil on 21. 6. 18..
//

#ifndef EOS_SCAN_TO_HUMAN_ART_IMGUI_H
#define EOS_SCAN_TO_HUMAN_ART_IMGUI_H

#include <artsim/artsim.h>

#include <glm/gtc/type_ptr.hpp>
#include <fmt/core.h>
#include <imgui.h>

namespace artsim {

void articulated_body_imgui(ArticulatedBody& art, bool& pos_edited, bool& vel_edited, bool& force_edited) {
    pos_edited = false;
    auto& art_spec = art.get_spec();
    real* pos_buf = art.get_pos_buf();
    real* vel_buf = art.get_vel_buf();
    real* force_buf = art.get_internal_force_buf();
    if (ImGui::TreeNode("Positions##art_pos")) {
        double pos_min = -5, pos_max = 5;
        double rot_min = -2*M_PI, rot_max = 2*M_PI;
        double quat_min = -1, quat_max = 1;
        for (int i = 0; i < art_spec.get_num_joints(); i++) {
            auto& joint = art_spec.joints[i];
            auto& joint_name = art_spec.names[i];
            int jidx_start = art_spec.joint_pos_dof_starts[i];
            auto label = fmt::format("{}##jointpos_{}", joint_name, joint_name);
            switch (joint.type) {
                JOINT_DOF_1_CASE {
                    pos_edited |= ImGui::DragFloat(label.c_str(), pos_buf + jidx_start, 0.01f, rot_min, rot_max);
                } break;
                case JOINT_TYPE_SPHERICAL: {
                    bool rot_edited = ImGui::DragFloat4(label.c_str(), pos_buf + jidx_start, 0.01f, quat_min, quat_max);
                    pos_edited |= rot_edited;
                    if (rot_edited) {
                        glm::rquat q = glm::normalize(glm::make_quat(pos_buf + jidx_start));
                        std::memcpy(pos_buf + jidx_start, (real*)&q, 4*sizeof(real));
                    }
                } break;
                case JOINT_TYPE_FLOATING: {
                    pos_edited |= ImGui::DragFloat3("Root pos##jointpos_root_pos", pos_buf + jidx_start, 0.01f, pos_min, pos_max);
                    bool rot_edited = ImGui::DragFloat4("Root rot##jointpos_root_rot", pos_buf + jidx_start + 3, 0.01f, quat_min, quat_max);
                    pos_edited |= rot_edited;
                    if (rot_edited) {
                        glm::rquat q = glm::normalize(glm::make_quat(pos_buf + jidx_start));
                        std::memcpy(pos_buf + jidx_start + 3, (real*)&q, 4*sizeof(real));
                    }
                } break;
            }
        }
        ImGui::TreePop();
        if (pos_edited) {
            art.forward_kinematics();
        }
    }
    if (ImGui::TreeNode("Velocities##art_vel")) {
        double vel_min = -2 * M_PI;
        double vel_max = 2 * M_PI;
        for (int i = 0; i < art_spec.get_num_joints(); i++) {
            auto& joint = art_spec.joints[i];
            auto& joint_name = art_spec.names[i];
            int jidx_start = art_spec.joint_vel_dof_starts[i];
            auto label = fmt::format("{}##jointvel_{}", joint_name, joint_name);
            switch (joint.type) {
                JOINT_DOF_1_CASE {
                    ImGui::DragFloat(label.c_str(), vel_buf + jidx_start, 0.01f, vel_min, vel_max);
                } break;
                case JOINT_TYPE_SPHERICAL: {
                    ImGui::DragFloat3(label.c_str(), vel_buf + jidx_start, 0.01f, vel_min, vel_max);
                } break;
                case JOINT_TYPE_FLOATING: {
                    ImGui::DragFloat3("Root vel##jointvel_root_vel", vel_buf + jidx_start, 0.01f, vel_min, vel_max);
                    ImGui::DragFloat3("Root angvel##jointvel_root_angvel", vel_buf + jidx_start + 3, 0.01f, vel_min, vel_max);
                } break;
            }
        }
        ImGui::TreePop();
    }
    if (ImGui::TreeNode("Force##art_force")) {
        double fmin = -1000;
        double fmax = 1000;
        for (int i = 0; i < art_spec.get_num_joints(); i++) {
            auto& joint = art_spec.joints[i];
            auto& joint_name = art_spec.names[i];
            int jidx_start = art_spec.joint_vel_dof_starts[i];
            auto label = fmt::format("{}##jointforce_{}", joint_name, joint_name);
            switch (joint.type) {
                JOINT_DOF_1_CASE {
                    ImGui::DragFloat(label.c_str(), force_buf + jidx_start, 0.01f, fmin, fmax);
                } break;
                case JOINT_TYPE_SPHERICAL: {
                    ImGui::DragFloat3(label.c_str(), force_buf + jidx_start, 0.01f, fmin, fmax);
                } break;
                case JOINT_TYPE_FLOATING: {
                    ImGui::DragFloat3("Root force##jointforce_root_force", force_buf + jidx_start, 0.01f, fmin, fmax);
                    ImGui::DragFloat3("Root torque##jointforce_root_torque", force_buf + jidx_start + 3, 0.01f, fmin, fmax);
                } break;
            }
        }
        ImGui::TreePop();
    }
}

}

#endif //EOS_SCAN_TO_HUMAN_ART_IMGUI_H
