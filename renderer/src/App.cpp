//
// Created by lasagnaphil on 2/6/18.
//

#include "gengine/App.h"
#include "gengine/InputManager.h"
#include "gengine/Arena.h"
#include "gengine/TrackballCamera.h"
#include "gengine/FlyCamera.h"

#ifdef GENGINE_USE_EGL
#include <glad/glad_egl.h>
#endif

#include <imgui.h>
#include <implot.h>
#include <backends/imgui_impl_sdl.h>
#include <backends/imgui_impl_opengl3.h>
#include <glm/ext.hpp>
#include <iostream>
#include <chrono>

#include "stb_image.h"
#include "stb_image_write.h"

#include <Tracy.hpp>
#include <SDL2/SDL_syswm.h>

static void sdl_die(const char * message) {
    fprintf(stderr, "%s: %s\n", message, SDL_GetError());
    exit(2);
}

void APIENTRY glDebugOutput(GLenum source,
                            GLenum type,
                            GLuint id,
                            GLenum severity,
                            GLsizei length,
                            const GLchar *message,
                            const void *userParam)
{
    // ignore non-significant error/warning codes
    if(id == 131169 || id == 131185 || id == 131218 || id == 131204) return;

    if (severity == GL_DEBUG_SEVERITY_NOTIFICATION) return;

    std::cout << "---------------" << std::endl;
    std::cout << "Debug message (" << id << "): " <<  message << std::endl;

    switch (source)
    {
        case GL_DEBUG_SOURCE_API:             std::cout << "Source: API"; break;
        case GL_DEBUG_SOURCE_WINDOW_SYSTEM:   std::cout << "Source: Window System"; break;
        case GL_DEBUG_SOURCE_SHADER_COMPILER: std::cout << "Source: Shader Compiler"; break;
        case GL_DEBUG_SOURCE_THIRD_PARTY:     std::cout << "Source: Third Party"; break;
        case GL_DEBUG_SOURCE_APPLICATION:     std::cout << "Source: Application"; break;
        case GL_DEBUG_SOURCE_OTHER:           std::cout << "Source: Other"; break;
    } std::cout << std::endl;

    switch (type)
    {
        case GL_DEBUG_TYPE_ERROR:               std::cout << "Type: Error"; break;
        case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR: std::cout << "Type: Deprecated Behaviour"; break;
        case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:  std::cout << "Type: Undefined Behaviour"; break;
        case GL_DEBUG_TYPE_PORTABILITY:         std::cout << "Type: Portability"; break;
        case GL_DEBUG_TYPE_PERFORMANCE:         std::cout << "Type: Performance"; break;
        case GL_DEBUG_TYPE_MARKER:              std::cout << "Type: Marker"; break;
        case GL_DEBUG_TYPE_PUSH_GROUP:          std::cout << "Type: Push Group"; break;
        case GL_DEBUG_TYPE_POP_GROUP:           std::cout << "Type: Pop Group"; break;
        case GL_DEBUG_TYPE_OTHER:               std::cout << "Type: Other"; break;
    } std::cout << std::endl;

    switch (severity)
    {
        case GL_DEBUG_SEVERITY_HIGH:         std::cout << "Severity: high"; break;
        case GL_DEBUG_SEVERITY_MEDIUM:       std::cout << "Severity: medium"; break;
        case GL_DEBUG_SEVERITY_LOW:          std::cout << "Severity: low"; break;
        case GL_DEBUG_SEVERITY_NOTIFICATION: std::cout << "Severity: notification"; break;
    } std::cout << std::endl;
    std::cout << std::endl;

    if (severity == GL_DEBUG_SEVERITY_HIGH) {
        assert(false);
        printf("Quitting program because of GL error!\n");
        exit(EXIT_FAILURE);
    }
}

void App::load() {
    // stb_image and stb_image_write settings
    stbi_set_flip_vertically_on_load(true);
    stbi_flip_vertically_on_write(true);

    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "Couldn't initialize SDL");
        exit(EXIT_FAILURE);
    }

    atexit(SDL_Quit);

    SDL_DisplayMode current;
    SDL_GetCurrentDisplayMode(0, &current);

    SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    window = SDL_CreateWindow("OpenGL", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                              settings.windowWidth, settings.windowHeight, window_flags);
    if (window == NULL) {
        fprintf(stderr, "Couldn't set video mode");
        exit(EXIT_FAILURE);
    }

    if (settings.useEGL) {
        if (!gladLoadEGL()) {
            fprintf(stderr, "Failed to load EGL with GLAD.\n");
            exit(EXIT_FAILURE);
        }

        glDisplay = eglGetDisplay(EGL_DEFAULT_DISPLAY);

        EGLint major, minor;

        EGLBoolean result = eglInitialize(glDisplay, &major, &minor);
        if(result != EGL_TRUE){
            switch(eglGetError()){
                case EGL_BAD_DISPLAY:
                    fprintf(stderr, "Failed to initialize EGL Display: EGL_BAD_DISPLAY");
                    exit(EXIT_FAILURE);
                case EGL_NOT_INITIALIZED:
                    fprintf(stderr, "Failed to initialize EGL Display: EGL_NOT_INITIALIZED");
                    exit(EXIT_FAILURE);
                default:
                    fprintf(stderr, "Failed to initialize EGL Display: unknown error");
                    exit(EXIT_FAILURE);
            }
        }

        // 2. Select an appropriate configuration
        static const EGLint configAttribs[] = {
                EGL_RED_SIZE,           8,
                EGL_GREEN_SIZE,         8,
                EGL_BLUE_SIZE,          8,
                EGL_ALPHA_SIZE,         8,
                EGL_DEPTH_SIZE,         24,
                EGL_STENCIL_SIZE,       8,
                EGL_RENDERABLE_TYPE,    EGL_OPENGL_BIT,
                EGL_SURFACE_TYPE,       EGL_PBUFFER_BIT,
                EGL_NONE
        };
        EGLint numConfigs;
        EGLConfig eglCfg;

        result = eglChooseConfig(glDisplay, configAttribs, &eglCfg, 1, &numConfigs);
        if(result != EGL_TRUE){
            switch(eglGetError()){
                case EGL_BAD_DISPLAY:
                    fprintf(stderr, "Failed to configure EGL Display: EGL_BAD_DISPLAY\n");
                    exit(EXIT_FAILURE);
                case EGL_BAD_ATTRIBUTE:
                    fprintf(stderr, "Failed to configure EGL Display: EGL_BAD_ATTRIBUTE\n");
                    exit(EXIT_FAILURE);
                case EGL_NOT_INITIALIZED:
                    fprintf(stderr, "Failed to configure EGL Display: EGL_NOT_INITIALIZED\n");
                    exit(EXIT_FAILURE);
                case EGL_BAD_PARAMETER:
                    fprintf(stderr, "Failed to configure EGL Display: EGL_BAD_PARAMETER\n");
                    exit(EXIT_FAILURE);
                default:
                    fprintf(stderr, "Failed to configure EGL Display: unknown error\n");
                    exit(EXIT_FAILURE);
            }
        }

        // 4. Bind the API
        result = eglBindAPI(EGL_OPENGL_API);
        if (result != EGL_TRUE) {
            fprintf(stderr, "Unable to bind OpenGL API (eglError: %d)\n", eglGetError());
            exit(EXIT_FAILURE);
        }

        EGLint ctxAttr[] = {
                EGL_CONTEXT_MAJOR_VERSION, 4,
                EGL_CONTEXT_MINOR_VERSION, 3,
                EGL_CONTEXT_OPENGL_PROFILE_MASK,
                EGL_CONTEXT_OPENGL_COMPATIBILITY_PROFILE_BIT,
                EGL_NONE
        };

        // Create a context
        glContext = eglCreateContext(glDisplay, eglCfg, EGL_NO_CONTEXT, ctxAttr);
        if (glContext == EGL_NO_CONTEXT) {
            fprintf(stderr, "Unable to create context (eglError: %d)\n", eglGetError());
            exit(EXIT_FAILURE);
        }

        // Create a window surface
        SDL_SysWMinfo sysInfo;
        SDL_VERSION(&sysInfo.version); // Set SDL version
        SDL_GetWindowWMInfo(window, &sysInfo);

        glSurface = eglCreateWindowSurface(glDisplay, eglCfg, (EGLNativeWindowType)sysInfo.info.x11.window, 0);
        if (glSurface == EGL_NO_SURFACE) {
            fprintf(stderr, "Unable to create EGL surface (eglError: %d)\n", eglGetError());
            exit(EXIT_FAILURE);
        }

        result = eglMakeCurrent(glDisplay, glSurface, glSurface, glContext);
        if (result != EGL_TRUE) {
            fprintf(stderr, "Unable to make surface as current (eglError: %d)\n", eglGetError());
            exit(EXIT_FAILURE);
        }
        eglSwapInterval(glDisplay, 1);

        if (!gladLoadGL()) {
            fprintf(stderr, "Cannot load GLAD!\n");
            exit(EXIT_FAILURE);
        }
    }
    else {
        SDL_GL_LoadLibrary(NULL); // Default OpenGL is fine.

#if __APPLE__
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
    #else
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    #endif
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
        SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
        SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
        SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

        // Create OpenGL Context
        glContext = SDL_GL_CreateContext(window);
        if (glContext == NULL)
            sdl_die("Failed to create OpenGL context");
        SDL_GL_SetSwapInterval(1); // Enable vsync

        // SDL Settings
        // SDL_SetRelativeMouseMode(isMouseRelative? SDL_TRUE : SDL_FALSE);

        // Check OpenGL properties
        printf("OpenGL loaded\n");
        gladLoadGLLoader(SDL_GL_GetProcAddress);
        printf("Vendor:   %s\n", glGetString(GL_VENDOR));
        printf("Renderer: %s\n", glGetString(GL_RENDERER));
        printf("Version:  %s\n", glGetString(GL_VERSION));

    }

    // Enable the debug callback
    GLint flags;
    glGetIntegerv(GL_CONTEXT_FLAGS, &flags);
    if (flags & GL_CONTEXT_FLAG_DEBUG_BIT) {
        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        glDebugMessageCallback(glDebugOutput, nullptr);
        glDebugMessageControl(
                GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, NULL, true
        );
    }

    // Set OpenGL viewport
    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    glViewport(0, 0, w, h);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

    // Setup ImGui binding
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    ImGui::StyleColorsDark();
    ImPlot::CreateContext();

    ImGui_ImplSDL2_InitForOpenGL(window, glContext);

#ifdef __APPLE__
    ImGui_ImplOpenGL3_Init("#version 410 core");
#else
    ImGui_ImplOpenGL3_Init("#version 130");
#endif

    rootTransform = Resources::make<Transform>();
    rootTransform->update();

    if (settings.camera == AppSettings::Camera::FlyCamera) {
        camera = std::make_unique<FlyCamera>(rootTransform, glm::ivec2(w, h));
    }
    else if (settings.camera == AppSettings::Camera::TrackballCamera) {
        camera = std::make_unique<TrackballCamera>(rootTransform);
    }

    if (settings.renderer == AppSettings::Renderer::Phong) {
        phongRenderer.setCamera(camera.get());
        phongRenderer.init();
    }
    else if (settings.renderer == AppSettings::Renderer::PBR) {
        pbRenderer.setCamera(camera.get());
        pbRenderer.init();
    }

    imRenderer.setCamera(camera.get());
    imRenderer.init();

    loadResources();
}

void App::startMainLoop() {
    using Clock = std::chrono::high_resolution_clock;
    using Time = decltype(Clock::now());
    using Ns = std::chrono::nanoseconds;
    using std::chrono::duration_cast;

    auto actualRender = [&]() {
        if (settings.useEGL) {
            auto result = eglMakeCurrent(glDisplay, glSurface, glSurface, glContext);
            if (result != EGL_TRUE) {
                fprintf(stderr, "Unable to make surface as current (eglError: %d)\n", eglGetError());
                exit(EXIT_FAILURE);
            }
        }
        internalRender();
        if (settings.useEGL) {
            eglSwapBuffers(glDisplay, glSurface);
        }
        else {
            SDL_GL_SwapWindow(window);
        };
    };

    Time previous = Clock::now();
    Ns lag = Ns(0);
    while (!quit) {
        Time current = Clock::now();
        Ns elapsed = duration_cast<Ns>(current - previous);
        previous = current;
        lag += elapsed;

        internalProcessInput();

        if (settings.useDisplayFPS) {
            Time previousUpdate = Clock::now();
            internalUpdate(elapsed.count() * 1e-9);
            dt = duration_cast<Ns>(Clock::now() - previousUpdate).count() * 1e-9;
        }
        else {
            uint64_t nsPerTick = 1e9 / settings.updateFPS;
            while (lag.count() > nsPerTick) {
                Time previousUpdate = Clock::now();

                // Update
                internalUpdate(nsPerTick * 1e-9f);
                if (!settings.skipRenderFramesOnLag) {
                    actualRender();
                    FrameMark
                }

                dt = duration_cast<Ns>(Clock::now() - previousUpdate).count() * 1e-9;
                lag -= Ns(nsPerTick);
            }
        }

        if (settings.skipRenderFramesOnLag) {
            actualRender();
            FrameMark
        }

        fps = (int)std::roundf(1.f / (duration_cast<Ns>(Clock::now() - current).count() * 1e-9));
    }
}

void App::loadResources() {

}

void App::processInput(SDL_Event& event) {

}

void App::update(float dt) {

}

void App::render() {

}

void App::release() {

}

void App::internalProcessInput() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        ImGui_ImplSDL2_ProcessEvent(&event);
        if (event.type == SDL_QUIT) {
            quit = true;
            break;
        } else if (event.type == SDL_KEYDOWN) {
            switch (event.key.keysym.sym) {
                case SDLK_ESCAPE:
                    quit = true;
                    break;
            }
        }
        camera->processInput(event);
        processInput(event);
    }
}

void App::internalUpdate(float dt) {
    auto inputMgr = InputManager::get();
    inputMgr->update();

    camera->update(dt);

    rootTransform->update();

    update(dt);
}

void App::internalRender() {
    glClear(GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame(window);
    ImGui::NewFrame();

    auto& io = ImGui::GetIO();

    // draw things
    render();

    ImGui::SetNextWindowBgAlpha(0.3f);
    ImGui::SetNextWindowPos(ImVec2(10.0f, 30.0f), ImGuiCond_Always, ImVec2(0.0f, 0.0f));
    bool overlayOpen = true;
    ImGui::Begin("Simple Overlay", &overlayOpen, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar);
    ImGui::Text("Update: %f ms", 1000.f * dt);
    ImGui::Text("Total FPS: %d", fps);
    if (settings.useDisplayFPS) {
        ImGui::Text("Update FPS: %d", settings.updateFPS);
    }
    ImGui::End();

    ImGui::Render();
    SDL_GL_MakeCurrent(window, glContext);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

App::~App() {
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    if (settings.useEGL) {
        eglDestroySurface(glDisplay, glSurface);
        eglDestroyContext(glDisplay, glContext);
        eglTerminate(glDisplay);
    }
    SDL_DestroyWindow(window);
}