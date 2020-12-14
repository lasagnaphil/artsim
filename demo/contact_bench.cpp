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

using real_t = double;

int main(int argc, char** argv)
{
    Camera camera;

    bool render = true;
    if (render) {
        // Initialization
        //--------------------------------------------------------------------------------------
        const int screenWidth = 1920;
        const int screenHeight = 1080;

        InitWindow(screenWidth, screenHeight, "Contact Benchmark");

        // Define the camera to look into our 3d world
        camera = { 0 };
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
    }

    ArticulatedBody art = examples::create_free_link(3, true);
    MaterialDB material_db;

    ArticulationState<real_t> state;

    auto reset = [&]() {
        ContactSolverType solver_type;
        if (strcmp(argv[1], "pgs") == 0) {
            solver_type = ContactSolverType::PGS;
        }
        else if (strcmp(argv[1], "bisection") == 0) {
            solver_type = ContactSolverType::Bisection;
        }
        else if (strcmp(argv[1], "ncp") == 0) {
            solver_type = ContactSolverType::NCP;
        }
        state = ArticulationState<real_t>(&art, &material_db, solver_type);
        state.enable_collision_with_ground = art.floating;
        state.randomize_positions();
    };
    reset();

    float dt = 1.0f / 600.0f;

    // Main game loop
    int counter = 0;
    int trials = 0;
    int test_duration = 180;
    int num_total_trials = 100;
    while ((render && !WindowShouldClose()) || !render)    // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        if (render) {
            auto& io = ImGui::GetIO();
            if (!io.WantCaptureMouse) {
                UpdateCamera(&camera);
            }
        }

        counter++;
        for (int i = 0; i < 10; i++) {
            auto t1 = std::chrono::high_resolution_clock::now();
            state.simulate(dt, 1);
            auto t2 = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1);
            printf("Duration: %ld ns\n", duration.count());
        }

        if (counter % test_duration == (test_duration - 1)) {
            reset();
            trials++;
            if (trials == num_total_trials) {
                break;
            }
        }

        if (render) {
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
        }
    }

    if (render) {
        ImGui_ImplRaylib_Shutdown();
        ImGui_ImplOpenGL3_Shutdown();
        ImPlot::DestroyContext();
        ImGui::DestroyContext();
        CloseWindow();        // Close window and OpenGL context
    }

    return 0;
}