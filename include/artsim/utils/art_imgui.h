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
                    pos_edited |= ImGui::SliderScalar(label.c_str(), ImGuiDataType_Real, pos_buf + jidx_start, &rot_min, &rot_max, "%.6g");
                } break;
                case JOINT_TYPE_SPHERICAL: {
                    bool rot_edited = ImGui::SliderScalarN(label.c_str(), ImGuiDataType_Real, pos_buf + jidx_start, 4, &quat_min, &quat_max, "%.6g");
                    pos_edited |= rot_edited;
                    if (rot_edited) {
                        glm::rquat q = glm::normalize(glm::make_quat(pos_buf + jidx_start));
                        std::memcpy(pos_buf + jidx_start, (real*)&q, 4*sizeof(real));
                    }
                } break;
                case JOINT_TYPE_FLOATING: {
                    pos_edited |= ImGui::SliderScalarN("Root pos##jointpos_root_pos", ImGuiDataType_Real, pos_buf + jidx_start, 3, &pos_min, &pos_max, "%.6g");
                    bool rot_edited = ImGui::SliderScalarN("Root rot##jointpos_root_rot", ImGuiDataType_Real, pos_buf + jidx_start + 3, 4, &quat_min, &quat_max, "%.6g");
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
                    ImGui::SliderScalar(label.c_str(), ImGuiDataType_Real, vel_buf + jidx_start, &vel_min, &vel_max, "%.6g");
                } break;
                case JOINT_TYPE_SPHERICAL: {
                    ImGui::SliderScalarN(label.c_str(), ImGuiDataType_Real, vel_buf + jidx_start, 3, &vel_min, &vel_max, "%.6g");
                } break;
                case JOINT_TYPE_FLOATING: {
                    ImGui::SliderScalarN("Root vel##jointvel_root_vel", ImGuiDataType_Real, vel_buf + jidx_start, 3, &vel_min, &vel_max, "%.6g");
                    ImGui::SliderScalarN("Root angvel##jointvel_root_angvel", ImGuiDataType_Real, vel_buf + jidx_start + 3, 3, &vel_min, &vel_max, "%.6g");
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
                    ImGui::SliderScalar(label.c_str(), ImGuiDataType_Real, force_buf + jidx_start, &fmin, &fmax, "%.6g");
                } break;
                case JOINT_TYPE_SPHERICAL: {
                    ImGui::SliderScalarN(label.c_str(), ImGuiDataType_Real, force_buf + jidx_start, 3, &fmin, &fmax, "%.6g");
                } break;
                case JOINT_TYPE_FLOATING: {
                    ImGui::SliderScalarN("Root force##jointforce_root_force", ImGuiDataType_Real, force_buf + jidx_start, 3, &fmin, &fmax, "%.6g");
                    ImGui::SliderScalarN("Root torque##jointforce_root_torque", ImGuiDataType_Real, force_buf + jidx_start + 3, 4, &fmin, &fmax, "%.6g");
                } break;
            }
        }
        ImGui::TreePop();
    }
}

}

#endif //EOS_SCAN_TO_HUMAN_ART_IMGUI_H
