//
// Created by Phillip Chang on 2020/09/26.
//

#include <chrono>
#include <random>

#include <artsim/artsim.h>
#include <artsim/soft_body.h>
#include <artsim/soft_body_dynamics.h>

#include <imgui.h>
#include <gengine/App.h>
#include <gengine/InputManager.h>
#include <gengine/FlyCamera.h>
#include <gengine_artsim/soft_body_render.h>

using namespace artsim;
using namespace glm;
using namespace glmx;

class MyApp : public App {
public:
    MyApp(const AppSettings& settings) : App(settings) {}

    void loadResources() {
        random_engine = std::default_random_engine(0);
        Eigen::setNbThreads(0);

        FlyCamera* camera = dynamic_cast<FlyCamera*>(this->camera.get());
        Ref<Transform> cameraTransform = camera->transform;
        cameraTransform->setGlobalPosition({0.0f, 5.0f, 10.0f});

        pbRenderer.dirLightProjVolume = {
                {-10.f, -10.f, 0.f}, {10.f, 10.f, 100.f}
        };
        pbRenderer.shadowFramebufferSize = {2048, 2048};

        pbRenderer.lights.dir.enabled = true;
        pbRenderer.lights.dir.direction = glm::normalize(glm::vec3 {2.0f, -3.0f, 2.0f});
        pbRenderer.lights.dir.color = glm::vec3(1.0f);

        Ref<PBRMaterial> soft_body_mat = PBRMaterial::quick(colors::Red);
        soft_body_mat->transparent = true;
        soft_body_mat->alpha = 0.8f;

        TetMesh tet_mesh;
        tet_mesh.load_obj("demo/resources/soft_body/octopus.obj");

        soft_body.load(tet_mesh);

        resetPhysics();

        soft_body_render = SoftBodyRender(&soft_body, soft_body_mat, camera);

        orig_mesh_mat = PBRMaterial::quick(colors::Green);

        imRenderer.reserveBuffers(0, 0, 16384);
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

            if (quasistatic) {
                if (sim_type == SimType::PD) {
                    projective_dynamics_quasistatic(soft_body, constraints_pd, num_iters,
                                                    (real*) sb_force.data(), INOUT (real*)sb_pos.data());
                }
                else {
                    printf("ADMM quasistatic not supported");
                    exit(EXIT_FAILURE);
                }
            }
            else {
                if (sim_type == SimType::PD) {
                    projective_dynamics(soft_body, constraints_pd, sim_dt, num_iters,
                                        (real*) sb_force.data(), INOUT (real*)sb_pos.data(), INOUT (real*)sb_vel.data());
                }
                else if (sim_type == SimType::ADMM) {
                    admm_dynamics(soft_body, constraints_admm, sim_dt, num_iters, (real*) sb_force.data(),
                                  INOUT (real*) sb_pos.data(), INOUT (real*) sb_vel.data());
                }
                /*
                else if (sim_type == SimType::QuasiNewton) {
                    quasinewton_dynamics(soft_body, soft_body_precalc, constraints, sim_dt, 5, (real*) sb_force.data(),
                                         INOUT (real*)sb_pos.data(), INOUT (real*)sb_vel.data());
                }
                */
            }

            auto t2 = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
            // printf("Duration: %lld microsecs\n", duration.count());

            // run_simulation = false;
        }
    }

    void render() override {
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        imRenderer.drawXZSquareGrid(-5.0f, 5.0f, 0.01f, 1.0f, colors::LightGray, true);

        soft_body_render.render(pbRenderer, sb_pos.data());
        // soft_body_render.render_debug_surface(imRenderer, sb_pos.data());

        if (sim_type == SimType::PD) {
            for (int i = 0; i < constraints_pd.positional.size(); i++) {
                imRenderer.drawSphere(constraints_pd.positional[i].target_pos, colors::Blue, 0.1f, false);
            }
        }
        else if (sim_type == SimType::ADMM) {
            for (int i = 0; i < constraints_admm.positional.size(); i++) {
                imRenderer.drawSphere(constraints_admm.positional[i].target_pos, colors::Blue, 0.1f, false);
            }
        }

        pbRenderer.render();
        imRenderer.render();

        ImGui::Begin("FEM Debug");
        if (ImGui::CollapsingHeader("Settings")) {
            if (ImGui::InputFloat("dt", &sim_dt)) resetPhysics();
            if (ImGui::Checkbox("quasistatic", &quasistatic)) {
                if (quasistatic) sim_type = SimType::PD;
                resetPhysics();
            }
            const char* sim_type_names[] = {"Projective Dynamics", "ADMM"};
            if (ImGui::ListBox("Sim Type", (int*)&sim_type, sim_type_names, quasistatic? 1 : 2)) resetPhysics();
        }
        if (ImGui::CollapsingHeader("Constraints")) {
            if (sim_type == SimType::PD) {
                for (int i = 0; i < constraints_pd.positional.size(); i++) {
                    real* pos = (real*)&constraints_pd.positional[i].target_pos;
                    real p_min = -10.0, p_max = 10.0;
                    std::string label = "Target pos " + std::to_string(i);
                    ImGui::SliderScalarN(label.c_str(), ImGuiDataType_Real, pos, 3, &p_min, &p_max);
                }
            }
            else if (sim_type == SimType::ADMM) {
                for (int i = 0; i < constraints_admm.positional.size(); i++) {
                    real* pos = (real*)&constraints_admm.positional[i].target_pos;
                    real p_min = -10.0, p_max = 10.0;
                    std::string label = "Target pos " + std::to_string(i);
                    ImGui::SliderScalarN(label.c_str(), ImGuiDataType_Real, pos, 3, &p_min, &p_max);
                }
            }
        }
        if (ImGui::CollapsingHeader("Positions")) {
            for (int i = 0; i < sb_pos.size(); i++) {
                auto v = sb_pos[i];
                ImGui::Text("%.6g\t%.6g\t%.6g", v.x, v.y, v.z);
            }
            ImGui::TreePop();
        }
        if (ImGui::CollapsingHeader("Velocities")) {
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
        sb_pos = soft_body.verts;
        real noise = 0.05;
        for (int i = 0; i < sb_pos.size(); i++) {
            sb_pos[i][0] += std::uniform_real_distribution<real>(-noise, noise)(random_engine);
            sb_pos[i][1] += std::uniform_real_distribution<real>(-noise, noise)(random_engine);
            sb_pos[i][2] += std::uniform_real_distribution<real>(-noise, noise)(random_engine);
        }
        sb_vel.clear();
        sb_vel.resize(sb_pos.size(), glm::tvec3<real>(0));
        sb_force.clear();
        sb_force.resize(sb_pos.size(), glm::tvec3<real>(0));

        SoftBodyProperties props;
        props.density = 1000;
        props.young_modulus = 1e7;
        props.poisson_ratio = 0.4;

        if (quasistatic) {
            soft_body.build_mass();
        }
        else {
            soft_body.build_mass(props.density, sim_dt);
        }

        if (sim_type == SimType::PD) {
            constraints_pd.linear_strain_energy.clear();
            constraints_pd.positional.clear();

            for (int i = 0; i < soft_body.tets.size(); i++) {
                constraints_pd.linear_strain_energy.push_back({i, 1e7, 1.0, 1.0});
                // constraints.volume_preservation_energy.push_back({i, 1e5, 0.9, 1.1});
            }
            constraints_pd.positional.push_back({0, 1e7, glm::rvec3(0, 0, 0)});
            constraints_pd.positional.push_back({400, 1e7, glm::rvec3(0, 0, 0)});

            for (auto& c : constraints_pd.linear_strain_energy) {
                soft_body.add_volume_constraint(c);
            }
            for (auto& c : constraints_pd.positional) {
                soft_body.add_positional_constraint(c);
            }
        }

        else if (sim_type == SimType::ADMM) {
            constraints_admm.corotational_energy.clear();
            constraints_admm.positional.clear();

            real stiffness = props.calc_corotational_stiffness();
            // real stiffness = 1e4;
            real mu = props.calc_mu();
            real lambda = props.calc_lambda();
            for (int i = 0; i < soft_body.tets.size(); i++) {
                constraints_admm.corotational_energy.push_back({i, stiffness, mu, lambda});
            }
            constraints_admm.positional.push_back({0, 1e7, glm::rvec3(0, 0, 0)});
            constraints_admm.positional.push_back({400, 1e7, glm::rvec3(0, 0, 0)});

            for (auto& c : constraints_admm.corotational_energy) {
                soft_body.add_volume_constraint(c);
            }
            for (auto& c : constraints_admm.positional) {
                soft_body.add_positional_constraint(c);
            }
        }

        soft_body.factorize();
    }

private:
    float sim_dt = 1.0f / 60.0f;
    int num_iters = 20;
    bool run_simulation = true;

    bool render_orig = false;

    bool quasistatic = false;
    enum class SimType : int {
        PD, ADMM
    } sim_type = SimType::PD;

    SoftBody soft_body;
    PDConstraints constraints_pd;
    ADMMConstraints constraints_admm;

    std::vector<glm::tvec3<real>> sb_pos;
    std::vector<glm::tvec3<real>> sb_vel;
    std::vector<glm::tvec3<real>> sb_force;

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