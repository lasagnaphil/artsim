//
// Created by Phillip Chang on 2020/09/26.
//

#include <chrono>
#include <random>

#include <artsim/artsim.h>
#include <artsim/soft_body.h>
#include <artsim/soft_body_with_art.h>
#include <artsim/articulation_state.h>

#include <imgui.h>
#include <implot.h>
#include <gengine/App.h>
#include <gengine/InputManager.h>
#include "soft_body_render.h"
#include "articulation_render.h"

using namespace artsim;
using namespace glm;
using namespace glmx;

class MyApp : public App {
public:
    MyApp(const AppSettings& settings) : App(settings) {}

    void loadResources() {
        random_engine = std::default_random_engine(0);
        Eigen::setNbThreads(16);

        FlyCamera* camera = dynamic_cast<FlyCamera*>(this->camera.get());
        Ref<Transform> cameraTransform = camera->transform;
        cameraTransform->move({0.0f, 2.0f, 0.0f});

        pbRenderer.dirLightProjVolume = {
                {-10.f, -10.f, 0.f}, {10.f, 10.f, 100.f}
        };
        pbRenderer.shadowFramebufferSize = {2048, 2048};

        pbRenderer.dirLight.enabled = true;
        pbRenderer.dirLight.direction = glm::normalize(glm::vec3 {2.0f, -3.0f, -2.0f});
        pbRenderer.dirLight.color = glm::vec3(1.0f);

        Ref<PBRMaterial> soft_body_mat = PBRMaterial::quick(colors::Red);
        soft_body_mat->alpha = 0.2f;

        soft_body_with_art.load("demo/resources/soft_body_with_art/metadata.xml");

        auto& props = soft_body_with_art.props;
        props.young_modulus = 1e8;
        props.poisson_ratio = 0.48;
        real stiffness = props.calc_corotational_stiffness();
        real mu = props.calc_mu();
        real lambda = props.calc_lambda();
        for (int i = 0; i < soft_body_with_art.tetrahedrons.size(); i++) {
            constraints.corotational_energy.push_back({i, stiffness, mu, lambda});
            // constraints.neohookean_energy.push_back({i, stiffness, mu, lambda});
        }
        soft_body_precomputation(soft_body_with_art, constraints, sim_dt);
        soft_body_render = SoftBodyRender(soft_body_with_art, soft_body_mat, camera);

        Ref<PBRMaterial> link_mat = PBRMaterial::quick(colors::Gray);
        Ref<PBRMaterial> joint_mat = PBRMaterial::quick(colors::Red);
        art_render = ArticulationRender(&soft_body_with_art.art, link_mat, joint_mat);

        resetPhysics();

        orig_mesh_mat = PBRMaterial::quick(colors::Green);
    }

    void processInput(SDL_Event &event) override {
    }

    void update(float dt) override {
        static float t = 0.0f;
        auto inputMgr = InputManager::get();

        if (inputMgr->isKeyEntered(SDL_SCANCODE_SPACE)) {
            run_simulation = !run_simulation;
        }
        if (inputMgr->isKeyEntered(SDL_SCANCODE_R)) {
            resetPhysics();
        }

        if (run_simulation) {
            auto t1 = std::chrono::high_resolution_clock::now();

            admm_dynamics_with_art(soft_body_with_art, constraints, sim_dt,
                                   (real*) sb_force.data(), (real*) art_force.data(),
                                   INOUT (real*)sb_pos.data(), INOUT (real*)sb_vel.data(),
                                   INOUT art_pos.data(), INOUT art_vel.data());

            auto t2 = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
            printf("Duration: %lld microsecs\n", duration.count());

            // run_simulation = false;

            t += sim_dt;
        }
    }

    void render() override {
        using namespace Eigen;
        using real = artsim::real;
        using MatrixXr = Matrix<real, Dynamic, Dynamic>;
        using VectorXr = Matrix<real, Dynamic, 1>;

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        imRenderer.drawXZSquareGrid(-5.0f, 5.0f, 0.01f, 1.0f, colors::LightGray, true);

        // soft_body_render.render(pbRenderer, sb_pos.data());
        soft_body_render.render_debug_surface(imRenderer, sb_pos.data());
        // soft_body_render.render_debug_volume(imRenderer, sb_pos.data());

        art_render.render(pbRenderer, art_pos.data());

        // render constrained vertex positions/velocities on soft body
        int N_f = soft_body_with_art.constrained_idx_start;
        int N_c = soft_body_with_art.vertices.size() - N_f;
        for (int i = N_f; i < N_f + N_c; i++) {
            imRenderer.drawPoint(sb_pos[i], colors::Green, 4.0f, true);
            imRenderer.drawArrow(sb_pos[i], sb_pos[i] + 0.1*sb_vel[i], colors::Green, 0.01f, true);
        }

        // render constrained vertex positions/velocities on articulation
        auto& sb_art = soft_body_with_art;
        std::vector<ttransform<real>> joint_trans(sb_art.art.get_num_joints());
        calc_transforms(sb_art.art, art_pos.data(), nullptr, joint_trans.data());
        Map<VectorXr> v_r(art_vel.data(), sb_art.art.get_num_vel_dofs());

        for (auto& [link_idx, vidx_range] : sb_art.constrained_vertices_range) {
            for (int vidx = vidx_range.first; vidx < vidx_range.second; vidx++) {
                auto offset = sb_art.constrained_vertices_offset[vidx];
                auto vert_trans = joint_trans[link_idx] * offset;
                imRenderer.drawPoint(vert_trans.v, colors::Blue, 4.0f, true);
            }
        }

        pbRenderer.render();
        imRenderer.render();

        ImGui::Begin("Debug");
        if (ImGui::CollapsingHeader("Soft Body")) {
            if (ImGui::TreeNode("Positions##sb_pos")) {
                for (int i = 0; i < sb_pos.size(); i++) {
                    auto v = sb_pos[i];
                    ImGui::Text("%.6g\t%.6g\t%.6g", v.x, v.y, v.z);
                }
                ImGui::TreePop();
            }
            if (ImGui::TreeNode("Velocities##sb_vel")) {
                for (int i = 0; i < sb_vel.size(); i++) {
                    auto v = sb_vel[i];
                    ImGui::Text("%.6g\t%.6g\t%.6g", v.x, v.y, v.z);
                }
                ImGui::TreePop();
            }
        }
        if (ImGui::CollapsingHeader("Articulation")) {
            if (ImGui::TreeNode("Positions##art_pos")) {
                double pos_min = -2*M_PI;
                double pos_max = 2*M_PI;
                for (int i = 0; i < art_pos.size(); i++) {
                    auto label = fmt::format("##art_pos_{}", i);
                    ImGui::SliderScalar(label.c_str(), ImGuiDataType_Double, &art_pos[i], &pos_min, &pos_max, "%.6g");
                }
                ImGui::TreePop();
            }
            if (ImGui::TreeNode("Velocities##art_vel")) {
                double vel_min = -2*M_PI;
                double vel_max = 2*M_PI;
                for (int i = 0; i < art_vel.size(); i++) {
                    auto label = fmt::format("##art_vel_{}", i);
                    ImGui::SliderScalar(label.c_str(), ImGuiDataType_Double, &art_vel[i], &vel_min, &vel_max, "%.6g");
                }
                ImGui::TreePop();
            }
            if (ImGui::TreeNode("Force##art_force")) {
                double fmin = -1000;
                double fmax = 1000;
                for (int i = 0; i < art_force.size(); i++) {
                    auto label = fmt::format("##art_force_{}", i);
                    ImGui::SliderScalar(label.c_str(), ImGuiDataType_Double, &art_force[i], &fmin, &fmax, "%.6g");
                }
                ImGui::TreePop();
            }
        }
        ImGui::End();
    }

    void release() override {
    }

    void resetPhysics() {
        sb_pos = soft_body_with_art.vertices;
        real noise = 0.0;
        for (int i = 0; i < soft_body_with_art.constrained_idx_start; i++) {
            sb_pos[i][0] += std::uniform_real_distribution<real>(-noise, noise)(random_engine);
            sb_pos[i][1] += std::uniform_real_distribution<real>(-noise, noise)(random_engine);
            sb_pos[i][2] += std::uniform_real_distribution<real>(-noise, noise)(random_engine);
        }
        sb_vel.clear();
        sb_vel.resize(sb_pos.size(), glm::tvec3<real>(0));
        sb_force.clear();
        sb_force.resize(sb_pos.size(), glm::tvec3<real>(0));
        sb_force_contact.clear();
        sb_force_contact.resize(sb_pos.size(), glm::tvec3<real>(0));

        int art_pos_dofs = soft_body_with_art.art.get_num_pos_dofs();
        int art_vel_dofs = soft_body_with_art.art.get_num_vel_dofs();
        art_pos.clear();
        art_pos.resize(art_pos_dofs);
        artsim::set_zero_pose(soft_body_with_art.art, art_pos.data());
        art_vel.clear();
        art_vel.resize(art_vel_dofs, 0);
        art_force.clear();
        art_force.resize(art_vel_dofs, 0);
        art_force_contact.clear();
        art_force_contact.resize(art_vel_dofs, 0);
    }

private:
    MaterialDB material_db;
    float sim_dt = 1.0f / 60.0f;
    bool run_simulation = false;
    bool render_orig = false;

    SoftBodyWithArtData soft_body_with_art;
    ADMMConstraints constraints;
    std::vector<glm::tvec3<real>> sb_pos, sb_vel, sb_force, sb_force_contact;
    std::vector<real> art_pos, art_vel, art_force, art_force_contact;

    SoftBodyRender soft_body_render;
    ArticulationRender art_render;

    Ref<PBRMaterial> ground_mat;
    Ref<Mesh> ground_mesh;

    Ref<PBRMaterial> orig_mesh_mat, joint_mat;
    int art_type = 1;

    std::default_random_engine random_engine;

    glm::tvec3<real> gravity = {0.0, -9.8, 0.0};
};

int main(int argc, char** argv)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    auto settings = AppSettings::defaultPBR();
    settings.useDisplayFPS = false;
    settings.updateFPS = 30;
    MyApp app(settings);
    app.load();
    app.startMainLoop();
    app.release();

    return 0;
}