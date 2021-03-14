//
// Created by Phillip Chang on 2020/09/26.
//

#include <chrono>

#include <artsim/artsim.h>
#include <artsim/articulation_state.h>
#include <artsim/utils/example_articulations.h>

#include <imgui.h>
#include <implot.h>
#include <gengine/App.h>
#include <gengine/InputManager.h>
#include "articulation_render.h"

using namespace artsim;
using namespace glm;
using namespace glmx;

class MyApp : public App {
public:
    MyApp() : App(AppSettings::defaultPBR()) {}

    enum class DemoType {
        Pendulum, Contacts
    };

    void loadResources() {

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

        ground_mat = PBRMaterial::quick(colors::White);
        ground_mesh = Mesh::makePlane(10.0f, 10.0f);

        resetPhysics();

        orig_mesh_mat = PBRMaterial::quick(0.5f * colors::Red);
        joint_mat = PBRMaterial::quick(colors::Green);
        art_render = ArticulationStateRender(&state, orig_mesh_mat, joint_mat);
    }

    void processInput(SDL_Event &event) override {
    }

    void update(float dt) override {
        auto inputMgr = InputManager::get();

        if (inputMgr->isKeyEntered(SDL_SCANCODE_R)) {
            resetPhysics();
        }
        if (inputMgr->isKeyEntered(SDL_SCANCODE_SPACE)) {
            run_simulation = !run_simulation;
        }
        if (inputMgr->isKeyEntered(SDL_SCANCODE_1)) { demo_type = DemoType::Pendulum; art_type = 1; resetPhysics(); }
        if (inputMgr->isKeyEntered(SDL_SCANCODE_2)) { demo_type = DemoType::Pendulum; art_type = 2; resetPhysics(); }
        if (inputMgr->isKeyEntered(SDL_SCANCODE_3)) { demo_type = DemoType::Pendulum; art_type = 3; resetPhysics(); }
        if (inputMgr->isKeyEntered(SDL_SCANCODE_4)) { demo_type = DemoType::Pendulum; art_type = 4; resetPhysics(); }
        if (inputMgr->isKeyEntered(SDL_SCANCODE_5)) { demo_type = DemoType::Pendulum; art_type = 5; resetPhysics(); }
        if (inputMgr->isKeyEntered(SDL_SCANCODE_6)) { demo_type = DemoType::Contacts; art_type = 1; resetPhysics(); }
        if (inputMgr->isKeyEntered(SDL_SCANCODE_7)) { demo_type = DemoType::Contacts; art_type = 2; resetPhysics(); }
        if (inputMgr->isKeyEntered(SDL_SCANCODE_8)) { demo_type = DemoType::Contacts; art_type = 3; resetPhysics(); }
        if (inputMgr->isKeyEntered(SDL_SCANCODE_9)) { demo_type = DemoType::Contacts; art_type = 4; resetPhysics(); }
        if (inputMgr->isKeyEntered(SDL_SCANCODE_0)) { demo_type = DemoType::Contacts; art_type = 5; resetPhysics(); }

        if (run_simulation) {
            auto t1 = std::chrono::high_resolution_clock::now();
            state.simulate(sim_dt, 10);
            auto t2 = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
            printf("Duration: %lld microsecs\n", duration.count());
        }
    }

    void render() override {
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (demo_type == DemoType::Contacts) {
            pbRenderer.queueRender({ground_mesh, ground_mat, rootTransform->getWorldTransform()});
        }
        art_render.render(pbRenderer, imRenderer);

        imRenderer.drawXZSquareGrid(-5.0f, 5.0f, 0.01f, 1.0f, colors::LightGray, true);

        pbRenderer.render();
        imRenderer.render();
    }

    void release() override {
    }

    void resetPhysics() {
        switch (demo_type) {
            case DemoType::Pendulum: {
                switch (art_type) {
                    case 1: art = examples::create_double_pendulum_ball(false, 1.0f, 1.0f, 1.0f, 1.0f); break;
                    case 2: art = examples::create_double_pendulum_link(false); break;
                    case 3: art = examples::create_triple_pendulum_link(false); break;
                    case 4: art = examples::create_furuta_pendulum(false); break;
                    case 5: art = examples::create_13_link_tree(true); break;
                }

                state = ArticulationState(&art, &material_db, ContactSolverType::PGS);
                state.enable_collision_with_ground = false;
                state.randomize_positions();

                art_render = ArticulationStateRender(&state, orig_mesh_mat, joint_mat);
            } break;
            case DemoType::Contacts: {
                art = examples::create_free_link(art_type, true);

                state = ArticulationState(&art, &material_db, ContactSolverType::PGS, 16);
                state.enable_collision_with_ground = art.floating;
                state.randomize_positions();

                art_render = ArticulationStateRender(&state, orig_mesh_mat, joint_mat);
            } break;

        }
    }

private:
    ArticulatedBody art;
    MaterialDB material_db;
    ArticulationState state;
    float sim_dt = 1.0f / 600.0f;
    bool run_simulation = true;

    Ref<PBRMaterial> ground_mat;
    Ref<Mesh> ground_mesh;

    Ref<PBRMaterial> orig_mesh_mat, joint_mat;
    ArticulationStateRender art_render;

    DemoType demo_type = DemoType::Pendulum;
    int art_type = 1;
};

int main(int argc, char** argv)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    MyApp app;
    app.load();
    app.startMainLoop();
    app.release();

    return 0;
}