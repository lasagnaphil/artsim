//
// Created by lasagnaphil on 2/16/21.
//

#ifndef ARTSIM_OBJ_FILE_H
#define ARTSIM_OBJ_FILE_H

#include <artsim/types.h>
#include <vector>
#include <unordered_map>

namespace artsim {

struct OBJFile {
    std::vector<glm::tvec3<real>> vertices;
    std::vector<glm::tvec3<real>> normals;
    std::vector<glm::tvec2<real>> uvs;

    std::vector<glm::ivec3> triangle_vertices;
    std::vector<glm::ivec3> triangle_normals;
    std::vector<glm::ivec3> triangle_uvs;

    std::vector<glm::ivec4> tetrahedrons;

    static OBJFile make_cube_tetrahedral(real dL, glm::ivec3 N);

    void load(const char* filename);
    void save(const char* filename);
};

}

#endif //ARTSIM_OBJ_FILE_H
