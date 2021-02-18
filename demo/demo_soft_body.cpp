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

        OBJFile objfile;
        // objfile.load("resources/soft_body_with_art/mesh_carved_.mesh");
        objfile.load("resources/soft_body/octopus.obj");
        // objfile.load("resources/soft_body/starfish.obj");
        // objfile.load("resources/soft_body/link_.mesh");
        // objfile = OBJFile::make_cube_tetrahedral(0.5, {1, 1, 1});
        // PyMesh::MshLoader msh("resources/soft_body_with_art/mesh_carved_.msh");
        // PyMesh::MshLoader msh("resources/soft_body/link_.msh");

        SoftBodyProperties props;
        props.young_modulus = 1e8;
        props.poisson_ratio = 0.4;
        props.dt = sim_dt;
        soft_body.load(objfile, props);
        soft_body.add_neohookean_energy_full_body(1e4, props.calc_mu(), props.calc_lambda());
        // soft_body.add_corotational_energy_full_body(1e5, props.calc_mu(), props.calc_lambda());
        // soft_body.add_volume_preservation_energy_full_body(props.young_modulus, 1.0, 1.0);
        soft_body.precomputation();
        soft_body_render = SoftBodyRender(&soft_body, soft_body_mat);

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

            soft_body_dynamics(soft_body, FEMAlgorithmType::ADMM, sim_dt, (real*) force.data(),
                               INOUT (real*)pos.data(), INOUT (real*)vel.data());


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

        soft_body_render.render(pbRenderer, imRenderer, pos.data());

        pbRenderer.render();
        imRenderer.render();

        ImGui::Begin("FEM Debug");
        if (ImGui::TreeNode("Positions")) {
            for (int i = 0; i < pos.size(); i++) {
                auto v = pos[i];
                ImGui::Text("%.6g\t%.6g\t%.6g", v.x, v.y, v.z);
            }
            ImGui::TreePop();
        }
        if (ImGui::TreeNode("Velocities")) {
            for (int i = 0; i < vel.size(); i++) {
                auto v = vel[i];
                ImGui::Text("%.6g\t%.6g\t%.6g", v.x, v.y, v.z);
            }
            ImGui::TreePop();
        }
        ImGui::End();
    }

    void release() override {
    }

    void resetPhysics() {
        pos = soft_body.vertices;
        real noise = 0.01;
        /*
        for (int i = 0; i < pos.size(); i++) {
            pos[i] += std::uniform_real_distribution<real>(-noise, noise)(random_engine);
        }
         */
        vel.clear();
        vel.resize(pos.size(), glm::tvec3<real>(0));
        force.clear();
        force.resize(pos.size(), glm::tvec3<real>(0));
    }

private:
    ArticulatedBody art;
    MaterialDB material_db;
    ArticulationState state;
    float sim_dt = 1.0f / 60.0f;
    bool run_simulation = false;
    bool render_orig = false;

    SoftBodyData soft_body;
    std::vector<glm::tvec3<real>> pos;
    std::vector<glm::tvec3<real>> vel;
    std::vector<glm::tvec3<real>> force;

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