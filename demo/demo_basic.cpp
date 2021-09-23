//
// Created by Phillip Chang on 2020/09/26.
//

#include <chrono>

#include <artsim/artsim.h>
#include <artsim/utils/example_articulations.h>
#include <artsim/utils/art_imgui.h>

#include <imgui.h>
#include <implot.h>
#include <gengine/App.h>
#include <gengine/FlyCamera.h>
#include <gengine/InputManager.h>
#include <gengine_artsim/articulation_render.h>
#include <gengine_artsim/rigid_body_render.h>
#include <gengine_artsim/world_debug_render.h>

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
        if (inputMgr->isKeyEntered(SDL_SCANCODE_RETURN)) {
            if (art_type == 1) {
                auto spec = RigidBodySpec(CollisionShape::make_sphere(0.1f), 1000);
                auto new_rb_id = world.add_rigid_body(spec, default_mat_id);
                auto& rb = *world.get_rigid_body(new_rb_id);
                rb.randomize_positions();
                rb_renderers.push_back(RigidBodyRender(&world, new_rb_id, orig_mesh_mat));
            }
            else {
                auto new_art_id = world.add_articulated_body(
                        examples::create_free_link(art_type, true), default_mat_id);
                // auto new_art_id = world.add_articulated_body(
                //         examples::create_free_ball(0.1f), default_mat_id);

                auto& art = *world.get_articulated_body(new_art_id);
                art.randomize_positions();
                art_renderers.push_back(ArticulationRender(&world, new_art_id, orig_mesh_mat, joint_mat));
            }
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
                world.simulate();
            }
            auto t2 = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
            printf("Duration: %lld microsecs\n", duration.count());
            // run_simulation = false;
        }
    }

    void render() override {
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (demo_type == DemoType::Contacts) {
            pbRenderer.queueRender({ground_mesh, ground_mat, rootTransform->getWorldTransform()});
        }
        for (auto& art_renderer : art_renderers) {
            art_renderer.render(pbRenderer);
        }
        for (auto& rb_renderer : rb_renderers) {
            rb_renderer.render(pbRenderer);
        }
        if (debug_render) {
            world_debug_renderer.render(imRenderer);
        }

        imRenderer.drawXZSquareGrid(-5.0f, 5.0f, 0.01f, 1.0f, colors::LightGray, true);
        /*
        for (auto& contact_point : state.contact_points) {
            imRenderer.drawSphere(contact_point.pos, colors::Red, 0.01f, false);
        }
        */
        ImGui::Begin("Debug");
        if (ImGui::CollapsingHeader("Renderer")) {
            ImGui::Checkbox("Enable debug render", &debug_render);
        }
        if (!art_id.is_null() && ImGui::CollapsingHeader("Articulation")) {
            auto& art = *world.get_articulated_body(art_id);
            auto [pos_edited, vel_edited, acc_edited] = articulated_body_imgui(art);
        }
        if (!rb_id.is_null() && ImGui::CollapsingHeader("Rigid Body")) {
            auto& rb = *world.get_rigid_body(rb_id);
            ImGui::DragScalarN("pos##rb_pos", ImGuiDataType_Real, &rb.pos, 3, 0.01f);
            if (ImGui::DragScalarN("rot##rb_rot", ImGuiDataType_Real, &rb.rot, 4, 0.01f)) {
                rb.rot = glm::normalize(rb.rot);
            }
            ImGui::DragScalarN("vel##rb_vel", ImGuiDataType_Real, &rb.vel, 3, 0.01f);
            ImGui::DragScalarN("angvel##rb_angvel", ImGuiDataType_Real, &rb.angvel, 3, 0.01f);
            ImGui::DragScalarN("acc##rb_acc", ImGuiDataType_Real, &rb.acc, 3, 0.01f);
            ImGui::DragScalarN("angacc##rb_angacc", ImGuiDataType_Real, &rb.angacc, 3, 0.01f);
            ImGui::DragScalarN("f_c##rb_f_c", ImGuiDataType_Real, &rb.f_c, 3, 0.01f);
        }
        ImGui::End();

        pbRenderer.render();
        imRenderer.render();
    }

    void release() override {
    }

    void resetPhysics() {
        art_id = {}; rb_id = {};
        art_renderers.clear();
        rb_renderers.clear();

        world = World();
        WorldConfig world_cfg;
        world_cfg.dt = sim_dt;
        world_cfg.max_vel_iters = 8;
        world_cfg.max_pos_iters = 2;
        world.init(world_cfg);

        default_mat_id = world.add_material(0.5f, 0.0f, 0.00f);

        switch (demo_type) {
            case DemoType::Pendulum: {
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
                art_renderers.push_back(ArticulationRender(&world, art_id, orig_mesh_mat, joint_mat));
            } break;
            case DemoType::Contacts: {
                world.add_plane(default_mat_id);
                if (art_type == 1) {
                    auto spec = RigidBodySpec(CollisionShape::make_box({0.1f, 1.0f, 0.1f}), 1000);
                    rb_id = world.add_rigid_body(spec, default_mat_id);
                    auto& rb = *world.get_rigid_body(rb_id);
                    rb.randomize_positions();
                    rb_renderers.push_back(RigidBodyRender(&world, rb_id, orig_mesh_mat));
                }
                else {
                    art_id = world.add_articulated_body(
                            examples::create_free_link(art_type, true), default_mat_id);

                    auto art = world.get_articulated_body(art_id);
                    art->randomize_positions();
                    art_renderers.push_back(ArticulationRender(&world, art_id, orig_mesh_mat, joint_mat));
                }
            } break;
        }
        world_debug_renderer = WorldDebugRender(&world);
    }

private:
    World world;
    Id<ArticulatedBody> art_id = {};
    Id<RigidBody> rb_id = {};
    Id<Material> default_mat_id;
    Material material {1.0f, 0.0f, 0.01f};
    float sim_dt = 1.0f / 600.0f;
    bool run_simulation = true;
    bool debug_render = false;

    Ref<PBRMaterial> ground_mat;
    Ref<Mesh> ground_mesh;

    Ref<PBRMaterial> orig_mesh_mat, joint_mat;
    std::vector<ArticulationRender> art_renderers;
    std::vector<RigidBodyRender> rb_renderers;
    WorldDebugRender world_debug_renderer;

    DemoType demo_type = DemoType::Contacts;
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