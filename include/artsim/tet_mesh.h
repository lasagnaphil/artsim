//
// Created by lasagnaphil on 21. 3. 16..
//

#ifndef ARTSIM_TET_MESH_H
#define ARTSIM_TET_MESH_H

#include <artsim/types.h>
#include <vector>

namespace artsim {

struct TetMesh {
    std::vector<glm::tvec3<real>> vertices;
    std::vector<glm::ivec4> tetrahedrons;

    static TetMesh make_cube_tetrahedral(real dL, glm::ivec3 N);

    void load_msh(const char* filename);
    void load_obj(const char* filename);

    void save_obj(const char* filename);
    void save_msh(const char* filename);
};

}
#endif //ARTSIM_TET_MESH_H
