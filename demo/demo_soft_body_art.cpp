//
// Created by Phillip Chang on 2020/09/26.
//

#include <chrono>
#include <random>

#include <artsim/artsim.h>
#include <artsim/art_with_soft_bodies.h>
#include <artsim/art_state.h>

#include <imgui.h>
#include <implot.h>
#include <openglrecorder.h>
#include <gengine/App.h>
#include <gengine/InputManager.h>
#include "soft_body_render.h"
#include "articulation_render.h"

using namespace artsim;
using namespace glm;
using namespace glmx;

class MyApp : public App {
public:
    MyApp(const AppSettings& settings) : App(settings) {}

    void loadResources() {
        RecorderConfig cfg;
        cfg.m_triple_buffering = 1;
        cfg.m_record_audio = 1;
        cfg.m_width = 1920;
        cfg.m_height = 1080;
        cfg.m_video_format = OGR_VF_VP8;
        cfg.m_audio_format = OGR_AF_VORBIS;
        cfg.m_audio_bitrate = 112000;
        cfg.m_video_bitrate = 200000;
        cfg.m_record_fps = 60;
        cfg.m_record_jpg_quality = 90;

        ogrInitConfig(&cfg);
        ogrRegReadPixelsFunction(glReadPixels);
        ogrRegPBOFunctions(glGenBuffers, glBindBuffer, glBufferData,
                           glDeleteBuffers, glMapBuffer, glUnmapBuffer);
        ogrSetSavedName("record");

        random_engine = std::default_random_engine(0);
        Eigen::setNbThreads(16);

        FlyCamera* camera = dynamic_cast<FlyCamera*>(this->camera.get());
        Ref<Transform> cameraTransform = camera->transform;
        cameraTransform->setGlobalPosition({0.0f, 1.0f, 2.0f});
        camera->movementSpeed = 1.0f;
        camera->near = 0.01f;
        camera->fov = 60.0f;

        pbRenderer.dirLightProjVolume = {
                {-10.f, -10.f, 0.f}, {10.f, 10.f, 100.f}
        };
        pbRenderer.shadowFramebufferSize = {2048, 2048};

        pbRenderer.dirLight.enabled = true;
        pbRenderer.dirLight.direction = glm::normalize(glm::vec3 {2.0f, -3.0f, -2.0f});
        pbRenderer.dirLight.color = glm::vec3(1.0f);

        Ref<PBRMaterial> soft_body_mat = PBRMaterial::quick(glm::vec3(252.f, 3.f, 3.f) / 255.f);
        // soft_body_mat->alpha = 0.2f;

        // system.load("demo/resources/art_with_soft_bodies/metadata.xml");
        system.load("/home/lasagnaphil/data/musculoskeleton/export_arm/metadata.xml");

        soft_body_renderers.reserve(system.get_num_soft_bodies());
        for (auto& sb : system.get_soft_bodies()) {
            soft_body_renderers.push_back({&sb, soft_body_mat, camera});
        }

        Ref<PBRMaterial> link_mat = PBRMaterial::quick(colors::LightGray);
        Ref<PBRMaterial> joint_mat = PBRMaterial::quick(colors::Blue);
        art_render = ArticulationRender(&system.get_articulation(), link_mat, joint_mat);

        resetPhysics();

        orig_mesh_mat = PBRMaterial::quick(colors::Green);

        soft_body_selection_mask.resize(system.get_num_soft_bodies(), false);
    }

    void processInput(SDL_Event &event) override {
    }

    void update(float dt) override {
        static float t = 0.0f;
        auto inputMgr = InputManager::get();

        if (inputMgr->isKeyEntered(SDL_SCANCODE_SPACE)) {
            run_simulation = !run_simulation;
            if (run_simulation) {
                ogrPrepareCapture();
            }
            else {
                ogrStopCapture();
            }
        }
        if (inputMgr->isKeyEntered(SDL_SCANCODE_R)) {
            resetPhysics();
        }

        if (run_simulation) {
            auto t1 = std::chrono::high_resolution_clock::now();

            system.integrate_admm_coupled();

            auto t2 = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
            printf("Duration: %lld microsecs\n", duration.count());

            // run_simulation = false;

            t += system.get_sim_deltatime();
        }
    }

    void render() override {
        using namespace Eigen;
        using real = artsim::real;
        using MatrixXr = Matrix<real, Dynamic, Dynamic>;
        using VectorXr = Matrix<real, Dynamic, 1>;

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        imRenderer.drawXZSquareGrid(-5.0f, 5.0f, 0.01f, 1.0f, colors::LightGray, true);

        auto& soft_bodies = system.get_soft_bodies();
        for (int i = 0; i < soft_bodies.size(); i++) {
            if (soft_body_selection_mask[i]) {
                soft_body_renderers[i].render(pbRenderer, (glm::rvec3*)system.get_soft_body_pos_buf(i));
                soft_body_renderers[i].render_debug_surface(imRenderer, (glm::rvec3*)system.get_soft_body_pos_buf(i));
                // soft_body_renderers[i].render_debug_volume(imRenderer, (glm::rvec3*)system.get_soft_body_pos_buf(i));
            }
        }

        glm::rvec3* pos_buf = system.get_soft_body_pos_buf(0);
        const int* constr_indices = system.get_soft_body_s_to_c_buf();
        for (int sb_idx = 0; sb_idx < system.get_num_soft_bodies(); sb_idx++) {
            if (soft_body_selection_mask[sb_idx]) {
                int sidx_start = system.get_soft_body_start_vidx(sb_idx);
                int sidx_end = system.get_soft_body_start_vidx(sb_idx+1);
                for (int sidx = sidx_start; sidx < sidx_end; sidx++) {
                    int cidx = constr_indices[sidx];
                    if (cidx != -1) {
                        glm::rvec3 pos = pos_buf[sidx];
                        imRenderer.drawPoint(pos, colors::Green, 4.0f, false);
                    }
                }
            }
        }

        art_render.render(pbRenderer, system.get_art_pos_buf());

        pbRenderer.render();
        imRenderer.render();

        ImGui::Begin("Debug");
        if (ImGui::CollapsingHeader("World Properties")) {
            double grav_min = -10;
            double grav_max = 10;
            glm::rvec3 gravity = system.get_gravity();
            if (ImGui::SliderScalarN("gravity", ImGuiDataType_Double, (real*)&gravity, 3, &grav_min, &grav_max)) {
                system.set_gravity(gravity);
            }
        }
        if (ImGui::CollapsingHeader("Articulation")) {
            if (ImGui::TreeNode("Positions##art_pos")) {
                double pos_min = -2*M_PI;
                double pos_max = 2*M_PI;
                real* art_pos = system.get_art_pos_buf();
                for (int i = 0; i < system.get_art_pos_dof(); i++) {
                    auto label = fmt::format("##art_pos_{}", i);
                    ImGui::SliderScalar(label.c_str(), ImGuiDataType_Double, &art_pos[i], &pos_min, &pos_max, "%.6g");
                }
                ImGui::TreePop();
            }
            if (ImGui::TreeNode("Velocities##art_vel")) {
                double vel_min = -2*M_PI;
                double vel_max = 2*M_PI;
                real* art_vel = system.get_art_vel_buf();
                for (int i = 0; i < system.get_art_vel_dof(); i++) {
                    auto label = fmt::format("##art_vel_{}", i);
                    ImGui::SliderScalar(label.c_str(), ImGuiDataType_Double, &art_vel[i], &vel_min, &vel_max, "%.6g");
                }
                ImGui::TreePop();
            }
            if (ImGui::TreeNode("Force##art_force")) {
                double fmin = -1000;
                double fmax = 1000;
                real* art_force = system.get_art_force_buf();
                for (int i = 0; i < system.get_art_vel_dof(); i++) {
                    auto label = fmt::format("##art_force_{}", i);
                    ImGui::SliderScalar(label.c_str(), ImGuiDataType_Double, &art_force[i], &fmin, &fmax, "%.6g");
                }
                ImGui::TreePop();
            }
        }
        if (ImGui::CollapsingHeader("Soft Body")) {
            if (ImGui::Button("Select/Deselect all")) {
                static bool all_selected = false;
                all_selected = !all_selected;
                std::fill(soft_body_selection_mask.begin(), soft_body_selection_mask.end(), all_selected);
            }
            for (int i = 0; i < system.get_num_soft_bodies(); i++) {
                std::string soft_body_label = std::string(system.get_soft_body_name(i));
                ImGui::Selectable(soft_body_label.c_str(), (bool*)&soft_body_selection_mask[i]);
            }
        }
        ImGui::End();

        if (run_simulation) {
            ogrCapture();
        }
    }

    void release() override {
        ogrDestroy();
    }

    void resetPhysics() {
        system.reset();

        /*
        art_root_trans = ttransform<real>(glm::rvec3(0, 1, 0));
        int sb_count = system.get_num_soft_bodies();
        auto& art = system.get_articulation();
        if (art.floating) {
            auto art_pos = system.get_art_pos_buf();
            *((glm::rvec3*)art_pos) = art_root_trans.v;
            *((glm::rquat*)(art_pos + 3)) = quat_cast(art_root_trans.R);
            for (int sb_idx = 0; sb_idx < sb_count; sb_idx++) {
                glm::rvec3* sb_pos = system.get_soft_body_pos_buf(sb_idx);
                sb_pos[sb_idx] = art_root_trans.v + art_root_trans.R * sb_pos[sb_idx];
            }
        }
         */
    }

private:
    MaterialDB material_db;
    bool run_simulation = false;
    bool render_orig = false;

    ArtWithSoftBodies system;

    std::vector<SoftBodyRender> soft_body_renderers;
    ArticulationRender art_render;
    std::vector<unsigned char> soft_body_selection_mask;

    Ref<PBRMaterial> ground_mat;
    Ref<Mesh> ground_mesh;

    Ref<PBRMaterial> orig_mesh_mat, joint_mat;
    int art_type = 1;

    std::default_random_engine random_engine;

    ttransform<real> art_root_trans;
};

int main(int argc, char** argv)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    auto settings = AppSettings::defaultPBR();
    settings.useDisplayFPS = false;
    settings.skipRenderFramesOnLag = true;
    settings.updateFPS = 60;
    MyApp app(settings);
    app.load();
    app.startMainLoop();
    app.release();

    return 0;
}