# add_library(raylib STATIC
#         raylib/src/core.c
#         raylib/src/shapes.c
#         raylib/src/textures.c
#         raylib/src/text.c
#         raylib/src/utils.c
#         raylib/src/rglfw.c
#         raylib/src/models.c
#         raylib/src/raudio.c)
#
# target_include_directories(raylib PUBLIC raylib/src)
#
# target_compile_definitions(raylib PRIVATE PLATFORM_DESKTOP GRAPHICS_API_OPENGL_33)
#
# target_compile_options(raylib PRIVATE -std=c99 -fno-strict-aliasing)
#
# target_link_libraries(raylib PUBLIC X11 pthread dl glfw)

set(PLATFORM Desktop CACHE STRING "" FORCE)
set(USE_AUDIO OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(OPENGL_VERSION 3.3 CACHE STRING "" FORCE)

add_subdirectory(raylib)

add_library(imgui STATIC
        imgui/imgui.cpp imgui/imgui_demo.cpp imgui/imgui_draw.cpp imgui/imgui_widgets.cpp
        imgui_impl_opengl3.cpp imgui_impl_raylib.cpp
        implot/implot.cpp implot/implot_items.cpp implot/implot_demo.cpp)

target_include_directories(imgui PUBLIC . imgui implot PRIVATE raylib/src raylib/src/external)
target_compile_definitions(imgui PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLAD)
