//
// Created by Phillip Chang on 2020/09/26.
//

#include <chrono>

#include <artsim/artsim.h>
#include <artsim/utils/example_articulations.h>

#include <imgui.h>
#include <implot.h>
#include <gengine/App.h>
#include <gengine/InputManager.h>
#include <gengine_artsim/articulation_render.h>

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

        pbRenderer.lights.dir.enabled = true;
        pbRenderer.lights.dir.direction = glm::normalize(glm::vec3 {2.0f, -3.0f, -2.0f});
        pbRenderer.lights.dir.color = glm::vec3(1.0f);

        ground_mat = PBRMaterial::quick(colors::White);
        ground_mesh = Mesh::makePlane(10.0f, 10.0f);

        resetPhysics();

        orig_mesh_mat = PBRMaterial::quick(0.5f * colors::Red);
        joint_mat = PBRMaterial::quick(colors::Green);
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
            for (int i = 0; i < 10; i++) {
                world.simulate(sim_dt);
            }
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
        art_render.render(pbRenderer);

        imRenderer.drawXZSquareGrid(-5.0f, 5.0f, 0.01f, 1.0f, colors::LightGray, true);
        /*
        for (auto& contact_point : state.contact_points) {
            imRenderer.drawSphere(contact_point.pos, colors::Red, 0.01f, false);
        }
        */

        pbRenderer.render();
        imRenderer.render();
    }

    void release() override {
    }

    void resetPhysics() {
        world = World();
        WorldConfig world_cfg;
        world_cfg.dt = sim_dt;
        world_cfg.max_iters = 8;
        world_cfg.contact_solver_type = ContactSolverType::PGS;
        default_mat_id = world.add_material();

        switch (demo_type) {
            case DemoType::Pendulum: {
                world_cfg.create_plane = false;
                world.init(world_cfg);
                switch (art_type) {
                    case 1: art_id = world.add_articulated_body(
                            examples::create_double_pendulum_ball(false, 1.0f, 1.0f, 1.0f, 1.0f), default_mat_id); break;
                    case 2: art_id = world.add_articulated_body(
                            examples::create_double_pendulum_link(false), default_mat_id); break;
                    case 3: art_id = world.add_articulated_body(
                            examples::create_triple_pendulum_link(false), default_mat_id); break;
                    case 4: art_id = world.add_articulated_body(
                            examples::create_furuta_pendulum(false), default_mat_id); break;
                    case 5: art_id = world.add_articulated_body(
                            examples::create_13_link_tree(true), default_mat_id); break;
                }

                auto art = world.get_articulated_body(art_id);
                art->randomize_positions();
            } break;
            case DemoType::Contacts: {
                world_cfg.create_plane = true;
                world.init(world_cfg);
                art_id = world.add_articulated_body(
                        examples::create_free_link(art_type, true), default_mat_id);

                auto art = world.get_articulated_body(art_id);
                art->randomize_positions();

            } break;
        }
        art_render = ArticulationRender(world.get_articulated_body(art_id), orig_mesh_mat, joint_mat);
    }

private:
    World world;
    Id<ArticulatedBody> art_id;
    Id<Material> default_mat_id;
    Material material {1.0f, 0.0f, 0.01f};
    float sim_dt = 1.0f / 600.0f;
    bool run_simulation = true;

    Ref<PBRMaterial> ground_mat;
    Ref<Mesh> ground_mesh;

    Ref<PBRMaterial> orig_mesh_mat, joint_mat;
    ArticulationRender art_render;

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