set_languages("cxx17")

function cmake_package(name, kind)
    package(name)
    add_deps("cmake")
    set_kind(kind)
    set_sourcedir(path.join(os.scriptdir(), name))
    on_install(function (package)
        local configs = {}
        table.insert(configs, "-DCMAKE_BUILD_TYPE=" .. (package:debug() and "Debug" or "Release"))
        table.insert(configs, "-DBUILD_SHARED_LIBS=" .. (package:config("shared") and "ON" or "OFF"))
        import("package.tools.cmake").install(package, configs)
    end)
    package_end()
end

includes("libs")

target("artsim")
    set_languages("cxx17")
    set_kind("static")
    add_files("src/*.cpp")
    add_files("src/math/*.cpp")
    add_files("src/anim/*.cpp")
    add_files("src/utils/*.cpp")
    add_files("src/utils/pymesh/*.cpp")
    add_includedirs("include", {public = true})
    add_packages("glm", "eigen", "tbb", "pugixml", "cereal", "fmt", {public = true})
    add_deps("bullet_collision", "span-lite", "fastsvd", "mshio", "tiny_obj_loader", "tracy")

includes("renderer")

includes("demo")
