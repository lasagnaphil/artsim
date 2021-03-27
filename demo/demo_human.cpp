//
// Created by lasagnaphil on 1/23/21.
//

#include <iostream>
#include <vector>
#include <tinyxml2.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <imgui.h>
#include <implot.h>

#include <artsim/artsim.h>
#include <artsim/art_state.h>
#include <artsim/utils/urdf.h>
#include <artsim/utils/xml.h>

#include "articulation_render.h"

#include <gengine/App.h>
#include <gengine/InputManager.h>

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

        pbRenderer.dirLight.enabled = true;
        pbRenderer.dirLight.direction = glm::normalize(glm::vec3 {2.0f, -3.0f, -2.0f});
        pbRenderer.dirLight.color = glm::vec3(1.0f);

        ground_mat = PBRMaterial::quick(colors::White);
        ground_mesh = Mesh::makePlane(10.0f, 10.0f);

        reset();

        orig_mesh_mat = PBRMaterial::quick(0.5f * colors::Red);
        joint_mat = PBRMaterial::quick(colors::Green);
        art_render = ArticulationStateRender(&state, orig_mesh_mat, joint_mat);
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
            state.simulate(sim_dt, 10);
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
        art_render.render(pbRenderer, imRenderer);

        imRenderer.drawXZSquareGrid(-5.0f, 5.0f, 0.01f, 1.0f, colors::LightGray, true);

        pbRenderer.render();
        imRenderer.render();
    }

    void release() override {
    }

    void reset() {
        art = load_from_xml("demo/resources/human.xml", contact_indices);
        // export_to_urdf(art, "human", "demo/resources/human.urdf");
        // art = load_from_xml("demo/resources/soft_body_with_art/two_link_art.xml", contact_indices);

        state = ArticulationState(&art, material, ContactSolverType::PGS, 16);
        state.enable_collision_with_ground = true;
        state.ground_col_enabled_links = contact_indices;
        state.set_root_transform(glmx::ttransform<real>(tvec3<real>(0.0f, 1.3f, 0.0f)));
        state.update_transforms();

        art_render = ArticulationStateRender(&state, orig_mesh_mat, joint_mat);
    }

private:
    ArticulatedBody art;
    ArticulationState state;
    Material material {1.0f, 0.0f, 0.01f};
    float sim_dt = 1.0f / 600.0f;
    bool run_simulation = true;

    std::vector<uint32_t> contact_indices;

    std::deque<float> prev_sim_times;

    Ref<PBRMaterial> ground_mat;
    Ref<Mesh> ground_mesh;

    Ref<PBRMaterial> orig_mesh_mat, joint_mat;
    ArticulationStateRender art_render;
};

int main(int argc, char** argv) {
    MyApp app;
    app.load();
    app.startMainLoop();
    app.release();
}
