# Threads
find_package(Threads REQUIRED)

# SDL2
find_package(SDL2 CONFIG REQUIRED)

# OpenGL
find_package(OpenGL CONFIG REQUIRED)

# GLAD
set(GLAD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/glad)
add_library(glad STATIC ${GLAD_DIR}/src/glad.c)
target_include_directories(glad PUBLIC ${GLAD_DIR}/include)
target_link_libraries(glad PUBLIC OpenGL::OpenGL)

add_library(glad_egl STATIC ${GLAD_EGL_DIR}/src/glad_egl.c)
target_include_directories(glad_egl PUBLIC ${GLAD_EGL_DIR}/include)
set(GLAD_EGL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/glad_egl)
target_link_libraries(glad_egl PUBLIC OpenGL::EGL)

# IMGUI
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/imgui)

add_library(imgui STATIC
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_demo.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp)
target_include_directories(imgui PUBLIC ${IMGUI_DIR})

# ImPlot
set(IMPLOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/implot)
add_library(implot STATIC
        ${IMPLOT_DIR}/implot.cpp
        ${IMPLOT_DIR}/implot_demo.cpp
        ${IMPLOT_DIR}/implot_items.cpp)
target_include_directories(implot PUBLIC ${IMPLOT_DIR})
target_link_libraries(implot PUBLIC imgui)

# OpenGLRecorder
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/deps/libopenglrecorder)

# STB
set(STB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/deps/stb)
add_library(stb INTERFACE)
target_include_directories(stb INTERFACE ${STB_DIR})

# Add the source files to the library
file(GLOB_RECURSE GENGINE_SOURCES src/*.cpp)

add_library(gengine STATIC ${GENGINE_SOURCES}
        ${IMGUI_DIR}/backends/imgui_impl_sdl.cpp
        ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp)

# include/linking
target_include_directories(gengine PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/generated
        ${SDL2_INCLUDE_DIR}/SDL2)

set(GENGINE_COMMON_LIBS artsim ${CMAKE_DL_LIBS} Threads::Threads 
        stb Eigen3::Eigen span-lite fmt::fmt imgui implot)

target_link_libraries(gengine PUBLIC ${GENGINE_COMMON_LIBS} glad glad_egl SDL2::Main openglrecorder)

target_compile_definitions(gengine PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLAD)
target_compile_definitions(gengine PUBLIC GENGINE_USE_EGL)

# Compile shaders into header files
add_executable(embed_shader ${CMAKE_CURRENT_SOURCE_DIR}/shaders/embed_shader.cpp)
target_link_libraries(embed_shader PUBLIC stdc++fs fmt::fmt)
file(GLOB shaders_src
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/*.vert"
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/*.frag"
        "${CMAKE_CURRENT_SOURCE_DIR}/shaders/*.geom")

set(SHADER_HEADER_FILES "")

file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/generated/shaders)

foreach(shader ${shaders_src})
    get_filename_component(shader_name ${shader} NAME)
    set(SHADER_HEADER_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/generated/shaders/${shader_name}.h)
    add_custom_command(
            OUTPUT ${SHADER_HEADER_OUTPUT_PATH}
            COMMAND embed_shader ${shader} ${SHADER_HEADER_OUTPUT_PATH}
            DEPENDS ${shader}
    )
    set(SHADER_HEADER_FILES ${SHADER_HEADER_FILES} ${SHADER_HEADER_OUTPUT_PATH})
endforeach()

add_custom_target(compile_shaders DEPENDS ${SHADER_HEADER_FILES})

add_dependencies(gengine compile_shaders)

