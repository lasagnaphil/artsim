//
// Created by lasagnaphil on 21. 8. 31..
//

#include <gengine/OBJLoader.h>

#include <fmt/core.h>
#include <tiny_obj_loader.h>
#include <filesystem>

namespace fs = std::filesystem;

OBJLoader::Object OBJLoader::loadPBRSingle(const char* filename) {
    tinyobj::ObjReaderConfig cfg;
    cfg.mtl_search_path = fs::path(filename).parent_path();

    tinyobj::ObjReader reader;
    if (!reader.ParseFromFile(filename, cfg)) {
        if (!reader.Error().empty()) {
            std::cerr << "TinyOBJReader: " << reader.Error();
        }
        exit(EXIT_FAILURE);
    }

    if (!reader.Warning().empty()) {
        std::cout << "TinyOBJReader: " << reader.Warning();
    }

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    auto& materials = reader.GetMaterials();

    auto mesh = Mesh::fromOBJ(attrib, shapes.data(), shapes.size());

    if (materials.size() == 0) {
        fmt::print("OBJLoader error in {}: Material not found!\n", filename);
        return {mesh, {}};
    }

    if (materials.size() > 1) {
        fmt::print("OBJLoader warning in {}: Too many materials! (using first one)\n", filename);
    }

    auto mat = PBRMaterial::fromOBJ(materials[0], cfg.mtl_search_path.c_str());

    return {mesh, mat};
}

