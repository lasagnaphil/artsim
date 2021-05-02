//
// Created by lasagnaphil on 3/1/21.
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
#include <artsim/utils/xml.h>
#include <artsim/ik.h>

#include <gengine/App.h>
#include <gengine/InputManager.h>

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
        art_render = ArticulationStateRender(&state, orig_mesh_mat, joint_mat);
    }

    void processInput(SDL_Event &event) override {
    }

    void update(float dt) override {
        auto inputMgr = InputManager::get();

        if (inputMgr->isKeyEntered(SDL_SCANCODE_R)) {
            reset();
        }

        if (inputMgr->isMouseEntered(SDL_BUTTON_LEFT)) {
            glm::vec2 mPos = inputMgr->getMousePos();
            Ray ray = camera->screenPointToRay(mPos);
            uint32_t leftHandIdx = 22;
            if (ray.intersectWithPlane(ikTargetPlane, ikTarget)) {
                inverse_kinematics(art, leftHandIdx, glmx::ttransform<real>(glmx::IDENTITY), ikTarget, state.q.data());
                state.update_transforms();
            }
        }
    }

    void render() override {
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        pbRenderer.queueRender({ground_mesh, ground_mat, rootTransform->getWorldTransform()});
        art_render.render(pbRenderer, imRenderer);

        imRenderer.drawXZSquareGrid(-5.0f, 5.0f, 0.01f, 1.0f, colors::LightGray, true);

        imRenderer.drawSphere(handPos, colors::Red, 0.1f, true);
        imRenderer.drawSphere(ikTarget, colors::Green, 0.1f, true);
        if (ikTargetPlane.dir.y == 0.0) {
            imRenderer.drawPlane(ikTargetPlane.pos, colors::Blue, ikTargetPlane.dir, colors::Blue, 1.0f, 0.1f, true);
        }

        pbRenderer.render();
        imRenderer.render();
    }

    void release() override {
    }

    void reset() {
        art = load_from_xml("demo/resources/human.xml", contact_indices);

        state = ArticulationState(&art, material, ContactSolverType::PGS, 16);
        state.enable_collision_with_ground = false;
        state.ground_col_enabled_links = contact_indices;
        state.set_root_transform(glmx::ttransform<real>(tvec3<real>(0.0f, 1.3f, 0.0f)));
        state.update_transforms();

        art_render = ArticulationStateRender(&state, orig_mesh_mat, joint_mat);
    }

private:
    ArticulatedBody art;
    Material material {1.0f, 0.0f, 0.01f};
    ArticulationState state;
    float sim_dt = 1.0f / 600.0f;
    bool run_simulation = true;

    std::vector<uint32_t> contact_indices;

    std::deque<float> prev_sim_times;

    Ref<PBRMaterial> ground_mat;
    Ref<Mesh> ground_mesh;

    Ref<PBRMaterial> orig_mesh_mat, joint_mat;
    ArticulationStateRender art_render;

    glm::vec3 handPos;
    glm::vec3 ikTarget = {0, 0, 0};
    Ray ikTargetPlane = {{0, 1, 0.4}, {0, 0, 1}};
};

int main(int argc, char** argv) {
    MyApp app;
    app.load();
    app.startMainLoop();
    app.release();
}
