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
        pbRenderer.dirLight.direction = glm::normalize(glm::vec3 {2.0f, -3.0f, 2.0f});
        pbRenderer.dirLight.color = glm::vec3(1.0f);

        Ref<PBRMaterial> soft_body_mat = PBRMaterial::quick(colors::Red);

        soft_body_with_art.load("demo/resources/soft_body_with_art/metadata.xml");

        auto& props = soft_body_with_art.sb.props;
        props.young_modulus = 1e8;
        props.poisson_ratio = 0.499;
        real stiffness = props.calc_corotational_stiffness();
        real mu = props.calc_mu();
        real lambda = props.calc_lambda();
        for (int i = 0; i < soft_body_with_art.sb.tetrahedrons.size(); i++) {
            constraints.corotational_energy.push_back({i, stiffness, mu, lambda});
            // constraints.neohookean_energy.push_back({i, stiffness, mu, lambda});
        }
        soft_body_precomputation(soft_body_with_art, constraints, sim_dt);
        soft_body_render = SoftBodyRender(&soft_body_with_art.sb, soft_body_mat);

        resetPhysics();

        orig_mesh_mat = PBRMaterial::quick(colors::Green);
    }

    void processInput(SDL_Event &event) override {
    }

    void update(float dt) override {
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
                                   INOUT (real*)sb_pos.data(), INOUT (real*)sb_vel.data(), INOUT (real*)sb_force_contact.data(),
                                   INOUT art_pos.data(), INOUT art_vel.data(), INOUT art_force_contact.data());

            auto t2 = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
            // printf("Duration: %lld microsecs\n", duration.count());

            run_simulation = false;
        }
    }

    void render() override {
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        imRenderer.drawXZSquareGrid(-5.0f, 5.0f, 0.01f, 1.0f, colors::LightGray, true);

        soft_body_render.render(pbRenderer, sb_pos.data());
        soft_body_render.render_debug_surface(imRenderer, sb_pos.data());

        pbRenderer.render();
        imRenderer.render();

        ImGui::Begin("FEM Debug");
        if (ImGui::TreeNode("Positions")) {
            for (int i = 0; i < sb_pos.size(); i++) {
                auto v = sb_pos[i];
                ImGui::Text("%.6g\t%.6g\t%.6g", v.x, v.y, v.z);
            }
            ImGui::TreePop();
        }
        if (ImGui::TreeNode("Velocities")) {
            for (int i = 0; i < sb_vel.size(); i++) {
                auto v = sb_vel[i];
                ImGui::Text("%.6g\t%.6g\t%.6g", v.x, v.y, v.z);
            }
            ImGui::TreePop();
        }
        ImGui::End();
    }

    void release() override {
    }

    void resetPhysics() {
        sb_pos = soft_body_with_art.sb.vertices;
        real noise = 0.02;
        for (int i = 0; i < sb_pos.size(); i++) {
            sb_pos[i] += std::uniform_real_distribution<real>(-noise, noise)(random_engine);
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
    ArticulationState state;
    float sim_dt = 1.0f / 60.0f;
    bool run_simulation = false;
    bool render_orig = false;

    SoftBodyWithArtData soft_body_with_art;
    ADMMConstraints constraints;
    std::vector<glm::tvec3<real>> sb_pos, sb_vel, sb_force, sb_force_contact;
    std::vector<real> art_pos, art_vel, art_force, art_force_contact;

    SoftBodyRender soft_body_render;

    Ref<PBRMaterial> ground_mat;
    Ref<Mesh> ground_mesh;

    Ref<PBRMaterial> orig_mesh_mat, joint_mat;
    int art_type = 1;

    std::default_random_engine random_engine;
};

int main(int argc, char** argv)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    auto settings = AppSettings::defaultPBR();
    settings.useDisplayFPS = false;
    settings.updateFPS = 60;
    MyApp app(settings);
    app.load();
    app.startMainLoop();
    app.release();

    return 0;
}