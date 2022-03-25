-- add_requires("glm", "eigen", "cereal", "fmt", "tbb", "pugixml")
add_requires("glm", "eigen", "fmt", "tbb", "pugixml")

cmake_package("cereal", "static")

target("bullet_collision")
    set_kind("static")
    add_files("bullet_collision/BulletCollision/**/*.cpp")
    add_files("bullet_collision/LinearMath/*.cpp")
    add_files("bullet_collision/LinearMath/**/*.cpp")
    add_includedirs("bullet_collision", {public = true})

target("span-lite")
    set_kind("headeronly")
    add_includedirs("span-lite", {public = true})

target("mshio")
    set_kind("static")
    add_files("MshIO/src/*.cpp")
    add_includedirs("MshIO/src")
    add_includedirs("MshIO/include", {public = true})

target("tiny_obj_loader")
    set_kind("static")
    add_files("tiny_obj_loader/tiny_obj_loader.cc")
    add_includedirs("tiny_obj_loader", {public = true})

target("tracy")
    set_kind("static")
    add_files("tracy/TracyClient.cpp")
    add_includedirs("tracy", {public = true})
    add_defines("TRACY_ENABLE")

includes("fastsvd")