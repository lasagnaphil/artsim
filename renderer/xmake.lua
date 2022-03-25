includes("deps")
includes("shaders")

target("gengine")
    set_kind("static")
    add_files("src/*.cpp")

    add_rules("compile_shaders")
    add_files("shaders/*.vert", "shaders/*.frag")

    add_files("deps/imgui/backends/imgui_impl_sdl.cpp")
    add_files("deps/imgui/backends/imgui_impl_opengl3.cpp")
    add_includedirs("include", {public = true})
    add_deps("artsim", "stb", "imgui", "implot", "glad", "glad_egl")
    add_packages("libsdl", "glm", "eigen", "fmt", {public = true})
