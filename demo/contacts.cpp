//
// Created by lasagnaphil on 20. 10. 17..
//

#include <chrono>

#include <raylib.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_raylib.h>
#include <implot.h>

#include <artsim/artsim.h>
#include <artsim/articulation_state.h>
#include <artsim/example_articulations.h>

#include "articulation_render.h"

using namespace artsim;
using namespace glm;

int main(void)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 1920;
    const int screenHeight = 1080;

    InitWindow(screenWidth, screenHeight, "pendulum");

    // Define the camera to look into our 3d world
    Camera camera = { 0 };
    camera.position = Vector3 { 0.0f, 5.0f, 5.0f };
    camera.target = Vector3 { 0.0f, 0.0f, 0.0f };
    camera.up = Vector3 { 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.type = CAMERA_PERSPECTIVE;
    // SetCameraMode(camera, CAMERA_THIRD_PERSON);
    SetCameraMode(camera, CAMERA_PERSPECTIVE);

    SetTargetFPS(60);               // Set our game to run at 60 frames-per-second

    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplOpenGL3_Init();
    ImGui_ImplRaylib_Init();

    //--------------------------------------------------------------------------------------

    ArticulatedBody art = examples::create_free_link(5, true);
    MaterialDB material_db;

    ArticulationState state;

    auto reset = [&]() {
        state = ArticulationState(&art, &material_db, ContactSolverType::NCP, 16);
        state.enable_collision_with_ground = art.floating;
        state.randomize_positions();
    };

    reset();

    float dt = 1.0f / 600.0f;

    bool run_simulation = true;

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        ttransform<real> rootT = state.get_root_transform();
        auto& io = ImGui::GetIO();
        if (!io.WantCaptureMouse) {
            UpdateCamera(&camera);
        }
        // SetCameraMode(camera, CAMERA_THIRD_PERSON);

        if (IsKeyPressed(KEY_R)) {
            reset();
        }
        if (IsKeyPressed(KEY_SPACE)) {
            run_simulation = !run_simulation;
        }

        if (run_simulation) {
            auto t1 = std::chrono::high_resolution_clock::now();
            state.simulate(dt, 10);
            auto t2 = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
            printf("Duration: %lld microsecs\n", duration.count());
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