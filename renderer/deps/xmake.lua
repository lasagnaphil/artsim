add_requires("libsdl")
add_requires("opengl")

target("stb")
    set_kind("headeronly")
    add_includedirs("stb", {public = true})

target("glad")
    set_kind("static")
    add_files("glad/src/glad.c")
    add_includedirs("glad/include", {public = true})
    add_packages("opengl")

target("glad_egl")
    set_kind("static")
    add_files("glad_egl/src/glad_egl.c")
    add_includedirs("glad_egl/include", {public = true})
    add_packages("opengl")

target("imgui")
    set_kind("static")
    add_files("imgui/*.cpp")
    add_includedirs("imgui", {public = true})

target("implot")
    set_kind("static")
    add_files("implot/*.cpp")
    add_includedirs("implot", {public = true})
    add_deps("imgui")

cmake_package("libopenglrecorder", "static")