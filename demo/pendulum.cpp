//
// Created by Phillip Chang on 2020/09/26.
//

#include "raylib.h"

#include <artsim/artsim.h>
#include <artsim/dynamics.h>
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
    camera.position = (Vector3){ 0.0f, 10.0f, 10.0f };
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.type = CAMERA_PERSPECTIVE;
    SetCameraMode(camera, CAMERA_THIRD_PERSON);

    SetTargetFPS(60);               // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------

    // ArticulatedBody art = examples::create_double_pendulum_ball(1.0f, 1.0f, 1.0f, 1.0f);
    // ArticulatedBody art = examples::create_double_pendulum_link();
    // ArticulatedBody art = examples::create_triple_pendulum_link();
    // ArticulatedBody art = examples::create_furuta_pendulum();
    ArticulatedBody art = examples::create_double_pendulum_link(true);

    ArticulationState state(&art);
    // state.randomize_positions();

    float dt = 1.0f / 240.0f;

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        UpdateCamera(&camera);

        state.simulate(dt, 4);

        //----------------------------------------------------------------------------------

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

        ClearBackground(RAYWHITE);

        BeginMode3D(camera);

        render_articulation(state);

        DrawGrid(10, 1.0f);        // Draw a grid

        EndMode3D();

        DrawFPS(10, 10);

        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}