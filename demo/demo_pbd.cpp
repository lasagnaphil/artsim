//
// Created by Phillip Chang on 2020/09/26.
//

#include <chrono>

#include <artsim/pbd.h>

#include <imgui.h>
#include <implot.h>
#include <gengine/App.h>
#include <gengine/InputManager.h>

using namespace artsim;
using namespace glm;
using namespace glmx;

class MyApp : public App {
public:
    MyApp(AppSettings settings) : App(settings) {}

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

        link_mat = PBRMaterial::quick(colors::Red);
        link_mesh = Mesh::makeCube({0.2f, 1.0f, 0.2f});

        resetPhysics();
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

        if (run_simulation) {
            auto t1 = std::chrono::high_resolution_clock::now();
            world.simulate(sim_dt, 20);
            auto t2 = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
            printf("Duration: %lld microsecs\n", duration.count());
        }
    }

    void render() override {
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        pbRenderer.queueRender({ground_mesh, ground_mat, rootTransform->getWorldTransform()});
        for (auto& link_id : links) {
            auto& rb = *world.get_rigid_body(link_id);
            glmx::transform trans = glmx::transform(rb.pos, glm::mat3_cast(rb.rot));
            glm::mat4 world_trans = mat4_cast(trans);
            pbRenderer.queueRender({link_mesh, link_mat, world_trans});
        }

        imRenderer.drawXZSquareGrid(-5.0f, 5.0f, 0.01f, 1.0f, colors::LightGray, true);

        pbRenderer.render();
        imRenderer.render();
    }

    void release() override {
        world.reset();
    }

    void resetPhysics() {
        world.reset();
        auto mat_id = world.make_material(1.0, 0.8, 0.2);
        auto plane_id = world.make_static_plane(mat_id, glm::vec3(0, 1, 0), 0);
        int link_group = 0b1000000;
        int link_mask = ~link_group;
        auto link1_id = world.make_cube(glm::rvec3(0.2, 1.0, 0.2), 10.0, mat_id, link_group, link_mask, glm::rvec3(0.0, 4.0, 0.0));
        auto link2_id = world.make_cube(glm::rvec3(0.2, 1.0, 0.2), 10.0, mat_id, link_group, link_mask, glm::rvec3(0.0, 3.0, 0.0));
        auto link3_id = world.make_cube(glm::rvec3(0.2, 1.0, 0.2), 10.0, mat_id, link_group, link_mask, glm::rvec3(0.0, 2.0, 0.0));
        auto link4_id = world.make_cube(glm::rvec3(0.2, 1.0, 0.2), 10.0, mat_id, link_group, link_mask, glm::rvec3(0.0, 1.0, 0.0));
        links.push_back(link1_id);
        links.push_back(link2_id);
        links.push_back(link3_id);
        links.push_back(link4_id);
        auto link1 = world.get_rigid_body(link1_id);
        auto link2 = world.get_rigid_body(link2_id);
        link2->rot = glmx::Rz(M_PI/8) * glmx::Rx(M_PI/10);
        auto link3 = world.get_rigid_body(link3_id);
        auto link4 = world.get_rigid_body(link4_id);

#if 0
        auto joint1_id = world.make_revolute_joint_constraint(
                link1_id, link2_id, 1e-8, glm::rvec3(0, -0.5, 0), glm::rvec3(0, 0.5, 0), glm::rvec3(0, 0, 1));
        auto joint2_id = world.make_revolute_joint_constraint(
                link2_id, link3_id, 1e-8, glm::rvec3(0, -0.5, 0), glm::rvec3(0, 0.5, 0), glm::rvec3(0, 0, 1));
        auto joint3_id = world.make_revolute_joint_constraint(
                link3_id, link4_id, 1e-8, glm::rvec3(0, -0.5, 0), glm::rvec3(0, 0.5, 0), glm::rvec3(0, 0, 1));
#else
        auto joint1_id = world.make_spherical_joint_constraint(
                link1_id, link2_id, 1e-8, glm::rvec3(0, -0.5, 0), glm::rvec3(0, 0.5, 0), glm::rvec3(0, -1, 0));
        auto joint2_id = world.make_spherical_joint_constraint(
                link2_id, link3_id, 1e-8, glm::rvec3(0, -0.5, 0), glm::rvec3(0, 0.5, 0), glm::rvec3(0, -1, 0));
        auto joint3_id = world.make_spherical_joint_constraint(
                link3_id, link4_id, 1e-8, glm::rvec3(0, -0.5, 0), glm::rvec3(0, 0.5, 0), glm::rvec3(0, -1, 0));
#endif

    }

private:
    Material material {1.0f, 0.0f, 0.01f};
    float sim_dt = 1.0f / 60.0f;
    bool run_simulation = false;

    std::vector<Id<PBDRigidBody>> links;

    Ref<PBRMaterial> ground_mat;
    Ref<Mesh> ground_mesh;

    Ref<PBRMaterial> link_mat;
    Ref<Mesh> link_mesh;

    PBDWorld world;
};

int main(int argc, char** argv)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    AppSettings settings = AppSettings::defaultPBR();
    settings.useDisplayFPS = false;
    settings.updateFPS = 60;
    MyApp app(settings);
    app.load();
    app.startMainLoop();
    app.release();

    return 0;
}