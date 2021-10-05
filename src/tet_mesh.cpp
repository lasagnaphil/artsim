//
// Created by lasagnaphil on 21. 3. 16..
//

#include "artsim/tet_mesh.h"
#include "artsim/utils/pymesh/MshLoader.h"
#include "artsim/utils/pymesh/MshSaver.h"

#include <fmt/core.h>
#include <fstream>
#include <sstream>
#include <iostream>

namespace artsim {

TetMesh TetMesh::make_cube_tetrahedral(real dL, glm::ivec3 N) {
    TetMesh obj;
    obj.vertices.resize((N.x+1)*(N.y+1)*(N.z+1));

#define INDEX(i,j,k) ((N.y+1)*(N.z+1)*(i) + (N.z+1)*(j) + (k))

    for (int i = 0; i <= N.x; i++) {
        for (int j = 0; j <= N.y; j++) {
            for (int k = 0; k <= N.z; k++) {
                obj.vertices[INDEX(i,j,k)] = -real(0.5) * dL * glm::rvec3(N) + glm::rvec3(i,j,k) * dL;
            }
        }
    }
    obj.tetrahedrons.reserve(6*N.x*N.y*N.z);
    for (int i = 0; i < N.x; i++) {
        for (int j = 0; j < N.y; j++) {
            for (int k = 0; k < N.z; k++) {
                obj.tetrahedrons.emplace_back(INDEX(i+0,j+0,k+0), INDEX(i+0,j+0,k+1), INDEX(i+0,j+1,k+1), INDEX(i+1,j+1,k+1));
                obj.tetrahedrons.emplace_back(INDEX(i+0,j+0,k+0), INDEX(i+0,j+0,k+1), INDEX(i+1,j+0,k+1), INDEX(i+1,j+1,k+1));
                obj.tetrahedrons.emplace_back(INDEX(i+0,j+0,k+0), INDEX(i+0,j+1,k+0), INDEX(i+0,j+1,k+1), INDEX(i+1,j+1,k+1));
                obj.tetrahedrons.emplace_back(INDEX(i+0,j+0,k+0), INDEX(i+0,j+1,k+0), INDEX(i+1,j+1,k+0), INDEX(i+1,j+1,k+1));
                obj.tetrahedrons.emplace_back(INDEX(i+0,j+0,k+0), INDEX(i+1,j+0,k+0), INDEX(i+1,j+0,k+1), INDEX(i+1,j+1,k+1));
                obj.tetrahedrons.emplace_back(INDEX(i+0,j+0,k+0), INDEX(i+1,j+0,k+0), INDEX(i+1,j+1,k+0), INDEX(i+1,j+1,k+1));
            }
        }
    }
    return obj;

#undef INDEX
}

void TetMesh::load_obj(const char* filename) {
    std::ifstream ifs(filename);
    std::string str;
    std::string index;
    std::stringstream ss;

    while (!ifs.eof()) {
        str.clear();
        index.clear();
        ss.clear();

        std::getline(ifs, str);
        ss.str(str);
        ss >> index;

        if (index == "v") {
            glm::tvec3<real> v;
            ss >> v[0] >> v[1] >> v[2];
            vertices.push_back(v);
        }
        else if (index == "t") {
            glm::ivec4 t;
            ss >> t[0] >> t[1] >> t[2] >> t[3];
            tetrahedrons.push_back(t);
        }
    }
    ifs.close();
}

void TetMesh::save_obj(const char* filename) {
    std::ofstream ofs(filename);

    for (auto& v : vertices) {
        ofs << "v " << v.x << " " << v.y << " " << v.z << std::endl;
    }
    for (auto& t : tetrahedrons) {
        ofs << "t " << t[0] << " " << t[1] << " " << t[2] << " " << t[3] << std::endl;
    }
}

void TetMesh::load_msh(const char* filename) {
    PyMesh::MshLoader msh(filename);
    auto& nodes = msh.get_nodes();
    auto& elems = msh.get_elements();
    int num_verts = nodes.rows() / 3;
    int num_tets = elems.rows() / 4;
    fmt::print("Loading msh {}: verts = {}, tets = {}\n", filename, num_verts, num_tets);
    vertices.resize(num_verts);
    std::copy_n(nodes.data(), nodes.size(), (real*)vertices.data());
    tetrahedrons.resize(num_tets);
    std::copy_n(elems.data(), elems.size(), (int*)tetrahedrons.data());

    // Fix tetrahedron sign (so that volume doesn't become negative);
    for (int tidx = 0; tidx < num_tets; tidx++) {
        auto& tet = tetrahedrons[tidx];
        auto v0 = vertices[tet[0]];
        auto v1 = vertices[tet[1]];
        auto v2 = vertices[tet[2]];
        auto v3 = vertices[tet[3]];
        auto D_m = glm::mat3(v1 - v0, v2 - v0, v3 - v0);
        auto det_D_m = glm::determinant(D_m);
        if (det_D_m < 0) {
            std::swap(tet[2], tet[3]);
        }
    }
}

void TetMesh::save_msh(const char* filename) {
    PyMesh::MshSaver msh(filename);
    Eigen::VectorXr nodes(3*vertices.size());
    Eigen::VectorXi elems(4*tetrahedrons.size());
    std::copy_n((real*)vertices.data(), nodes.size(), nodes.data());
    std::copy_n((int*)tetrahedrons.data(), elems.size(), elems.data());
    msh.save_mesh(nodes, elems, 3, PyMesh::MshSaver::ElementType::TET);
}

}

