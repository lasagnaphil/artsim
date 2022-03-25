//
// Created by lasagnaphil on 1/23/21.
//

#include <iostream>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <imgui.h>
#include <implot.h>

#include <artsim/artsim.h>
#include <artsim/utils/urdf.h>
#include <artsim/utils/xml.h>
#include <artsim/utils/art_imgui.h>

#include <gengine/App.h>
#include <gengine/InputManager.h>
#include <gengine/FlyCamera.h>
#include <gengine_artsim/articulation_render.h>


using namespace artsim;

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

        reset();

        orig_mesh_mat = PBRMaterial::quick(0.5f * colors::Red);
        joint_mat = PBRMaterial::quick(colors::Green);
    }

    void processInput(SDL_Event &event) override {
    }

    void update(float dt) override {
        auto inputMgr = InputManager::get();

        if (inputMgr->isKeyEntered(SDL_SCANCODE_R)) {
            reset();
        }
        if (inputMgr->isKeyEntered(SDL_SCANCODE_SPACE)) {
            run_simulation = !run_simulation;
        }

        if (run_simulation) {
            auto t1 = std::chrono::high_resolution_clock::now();
            for (int i = 0; i < 10; i++) {
                world.simulate();
            }
            auto t2 = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
            float sim_time = 0.001f * duration.count() / 10;
            if (prev_sim_times.size() == 60) {
                prev_sim_times.pop_front();
            }
            prev_sim_times.push_back(sim_time);
        }

        float avg_sim_time = 0.0f;
        for (auto& t : prev_sim_times) {
            avg_sim_time += t;
        }
        avg_sim_time /= prev_sim_times.size();
    }

    void render() override {
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        pbRenderer.queueRender({ground_mesh, ground_mat, rootTransform->getWorldTransform()});
        art_render.render(pbRenderer);

        imRenderer.drawXZSquareGrid(-5.0f, 5.0f, 0.01f, 1.0f, colors::LightGray, true);
        /*
        for (auto& contact_point : state.contact_points) {
            imRenderer.drawSphere(contact_point.pos, colors::Red, 0.01f, false);
        }
        */

        pbRenderer.render();
        imRenderer.render();

        auto art = world.get_articulated_body(art_id);
        auto [pos_edited, vel_edited, force_edited] = articulated_body_imgui(*art);
    }

    void release() override {
    }

    void reset() {
        world = World();
        WorldConfig world_cfg;
        world_cfg.dt = sim_dt;
        world_cfg.max_vel_iters = 8;
        world_cfg.max_pos_iters = 2;
        world.init(world_cfg);
        default_mat_id = world.add_material(1.0f, 0.0f, 0.00f);

        world.add_plane(default_mat_id);

        ArticulatedBodySpec art_spec;
        if (!load_from_xml("demo/resources/human.xml", art_spec)) {
            fmt::print("Failed to load articulation!\n");
            exit(EXIT_FAILURE);
        }
        /*
        if (save_to_xml("demo/resources/human.xml", art_spec)) {
            fmt::print("Failed to save articulation!\n");
            exit(EXIT_FAILURE);
        }
         */

        art_id = world.add_articulated_body(art_spec, default_mat_id);
        auto art = world.get_articulated_body(art_id);

        art->set_root_transform(glmx::ttransform<real>(tvec3<real>(0.0f, 1.3f, 0.0f)));
        art->forward_kinematics();

        art_render = ArticulationRender(&world, art_id, orig_mesh_mat, joint_mat);
    }

private:
    World world;
    Id<ArticulatedBody> art_id;
    Id<Material> default_mat_id;
    Material material {1.0f, 0.0f, 0.01f};
    float sim_dt = 1.0f / 600.0f;
    bool run_simulation = true;

    std::vector<uint32_t> contact_indices;

    std::deque<float> prev_sim_times;

    Ref<PBRMaterial> ground_mat;
    Ref<Mesh> ground_mesh;

    Ref<PBRMaterial> orig_mesh_mat, joint_mat;
    ArticulationRender art_render;
};

int main(int argc, char** argv) {
    MyApp app;
    app.load();
    app.startMainLoop();
    app.release();
}
