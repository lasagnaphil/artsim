//
// Created by Phillip Chang on 2020/09/26.
//

#include <chrono>

#include <raylib.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_raylib.h>
#include <implot.h>

#include <artsim/artsim.h>
#include <artsim/dynamics.h>
#include <artsim/articulation_state.h>
#include <artsim/example_articulations.h>

#include "articulation_render.h"

inline Vector3 glm_to_raylib(glm::vec3 v) {
    return (Vector3){v.x, v.y, v.z};
}

using namespace artsim;
using namespace glm;

using real_t = double;

int main(void)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 1920;
    const int screenHeight = 1080;

    InitWindow(screenWidth, screenHeight, "pendulum");

    // Define the camera to look into our 3d world
    Camera camera = { 0 };
    camera.position = (Vector3){ 0.0f, 5.0f, 5.0f };
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.type = CAMERA_PERSPECTIVE;
    SetCameraMode(camera, CAMERA_THIRD_PERSON);
    // SetCameraMode(camera, CAMERA_PERSPECTIVE);

    SetTargetFPS(60);               // Set our game to run at 60 frames-per-second

    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplOpenGL3_Init();
    ImGui_ImplRaylib_Init();

    //--------------------------------------------------------------------------------------

    // ArticulatedBody art = examples::create_double_pendulum_ball(false, 1.0f, 1.0f, 1.0f, 1.0f);
    // ArticulatedBody art = examples::create_double_pendulum_link(false);
    // ArticulatedBody art = examples::create_triple_pendulum_link(false);
    // ArticulatedBody art = examples::create_furuta_pendulum(false);
    // ArticulatedBody art = examples::create_13_link_tree(true);
    ArticulatedBody art = examples::create_free_link(3, true);
    MaterialDB material_db;

    ArticulationState<real_t> state(&art, &material_db);
    state.enable_collision_with_ground = art.floating;
    state.randomize_positions();
    auto T_root = state.get_root_transform();
    T_root.v.y += 3.0;
    state.set_root_transform(T_root);
    // state.set_joint_pos_1dof(0, 0.2f * 3.14f);
    // state.set_joint_pos_1dof(1, 0.3f * 3.14f);
    // state.set_joint_pos_spherical(0, glm::angleAxis(0.1f * glm::pi<float>(), glm::normalize(glm::vec3(1, 0, 1))));
    // state.set_joint_pos_spherical(1, glm::angleAxis(-0.1f * glm::pi<float>(), glm::normalize(glm::vec3(1, 0, 1))));

    float dt = 1.0f / 600.0f;

    bool run_simulation = true;

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        ttransform<real_t> rootT = state.get_root_transform();
        auto& io = ImGui::GetIO();
        if (!io.WantCaptureMouse) {
            UpdateCamera(&camera);
        }
        // SetCameraMode(camera, CAMERA_THIRD_PERSON);

        if (IsKeyPressed(KEY_R)) {
            state = ArticulationState<real_t>(&art, &material_db);
            state.enable_collision_with_ground = true;
            state.randomize_positions();
            auto T_root = state.get_root_transform();
            T_root.v.y += 3.0;
            state.set_root_transform(T_root);
        }
        if (IsKeyPressed(KEY_SPACE)) {
            run_simulation = !run_simulation;
        }

        if (run_simulation) {
            auto t1 = std::chrono::high_resolution_clock::now();
            state.simulate(dt, 10);
            auto t2 = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
            printf("Duration: %ld microsecs\n", duration.count());
        }

        //----------------------------------------------------------------------------------
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplRaylib_NewFrame();
        ImGui::NewFrame();
        ImGui_ImplRaylib_ProcessEvent();

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();
        {
            ClearBackground(RAYWHITE);
            BeginMode3D(camera);
            {
                render_articulation(state);
                DrawGrid(10, 1.0f);        // Draw a grid
            }
            EndMode3D();

            DrawFPS(10, 10);
            // ImGui::ShowDemoWindow();
            // ImPlot::ShowDemoWindow();
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        }
        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    ImGui_ImplRaylib_Shutdown();
    ImGui_ImplOpenGL3_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}