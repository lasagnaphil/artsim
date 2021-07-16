//
// Created by lasagnaphil on 2/16/21.
//

#include "artsim/obj_file.h"
#include "artsim/utils/pymesh/MshLoader.h"

#include <fmt/core.h>
#include <fstream>
#include <sstream>
#include <iostream>

namespace artsim {

OBJFile OBJFile::make_cube_tetrahedral(real dL, glm::ivec3 N) {
    OBJFile obj;
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

void OBJFile::load_obj(const char* filename) {
    std::ifstream ifs(filename);
    std::string str;
    std::string index;

    while (std::getline(ifs, str)) {
        std::istringstream ss(str);
        ss >> index;

        if (index == "v") {
            glm::tvec3<real> v;
            ss >> v[0] >> v[1] >> v[2];
            vertices.push_back(v);
        }
        else if (index == "vn") {
            glm::tvec3<real> vn;
            ss >> vn[0] >> vn[1] >> vn[2];
            normals.push_back(vn);
        }
        else if (index == "vt") {
            glm::tvec2<real> vt;
            ss >> vt[0] >> vt[1];
            uvs.push_back(vt);
        }
        else if (index == "f") {
            glm::ivec3 fi, ft, fn;
            int res;
            res = sscanf(str.c_str(), "f %d %d %d", &fi.x, &fi.y, &fi.z);
            if (res == 3) {
                triangle_vertices.push_back(fi-1);
                continue;
            }
            res = sscanf(str.c_str(), "f %d//%d %d//%d %d//%d",
                         &fi.x,&fn.x,
                         &fi.y,&fn.y,
                         &fi.z,&fn.z);
            if (res == 6) {
                triangle_vertices.push_back(fi-1);
                triangle_normals.push_back(fn-1);
                continue;
            }
            res = sscanf(str.c_str(), "f %d/%d/%d %d/%d/%d %d/%d/%d",
                         &fi.x,&ft.x,&fn.x,
                         &fi.y,&ft.y,&fn.y,
                         &fi.z,&ft.z,&fn.z);
            if (res == 9) {
                triangle_vertices.push_back(fi-1);
                triangle_uvs.push_back(ft-1);
                triangle_normals.push_back(fn-1);
                continue;
            }
            fprintf(stderr, "Error while parsing OBJ file!\n");
        }
        else if (index == "t") {
            glm::ivec4 t;
            ss >> t[0] >> t[1] >> t[2] >> t[3];
            tetrahedrons.push_back(t);
        }
    }
    ifs.close();
}

void OBJFile::save_obj(const char* filename) {
    std::ofstream ofs(filename);

    for (auto& v : vertices) {
        ofs << "v " << v.x << " " << v.y << " " << v.z << std::endl;
    }
    for (auto& vn : normals) {
        ofs << "vn " << vn.x << " " << vn.y << " " << vn.z << std::endl;
    }
    for (auto& vt : uvs) {
        ofs << "vt " << vt.x << " " << vt.y << std::endl;
    }
    if (!triangle_vertices.empty() && !triangle_uvs.empty() && !triangle_normals.empty() ) {
        for (int i = 0; i < triangle_vertices.size(); i++) {
            auto fi = triangle_vertices[i]+1;
            auto ft = triangle_uvs[i]+1;
            auto fn = triangle_normals[i]+1;
            ofs << "f " << fi[0] << "/" << ft[0] << "/" << fn[0] << " "
                        << fi[1] << "/" << ft[1] << "/" << fn[1] << " "
                        << fi[2] << "/" << ft[2] << "/" << fn[2] << std::endl;
        }
    }
    else if (!triangle_vertices.empty() && !triangle_normals.empty()) {
        for (int i = 0; i < triangle_vertices.size(); i++) {
            auto fi = triangle_vertices[i]+1;
            auto fn = triangle_normals[i]+1;
            ofs << "f " << fi[0] << "//" << fn[0] << " "
                        << fi[1] << "//" << fn[1] << " "
                        << fi[2] << "//" << fn[2] << std::endl;
        }
    }
    else if (!triangle_vertices.empty()) {
        for (int i = 0; i < triangle_vertices.size(); i++) {
            auto fi = triangle_vertices[i]+1;
            ofs << "f " << fi[0] << " " << fi[1] << " " << fi[2] << std::endl;
        }
    }
    for (auto& t : tetrahedrons) {
        ofs << "t " << t[0] << " " << t[1] << " " << t[2] << " " << t[3] << std::endl;
    }
}

void OBJFile::load_msh(const char* filename) {
    PyMesh::MshLoader msh(filename);
    auto& nodes = msh.get_nodes();
    auto& elems = msh.get_elements();
    int num_nodes = nodes.rows() / 3;
    int num_elems = elems.rows() / 4;
    vertices.resize(num_nodes);
    for (int i = 0; i < num_nodes; i++) {
        vertices[i] = {nodes[3*i+0], nodes[3*i+1], nodes[3*i+2]};
    }
    tetrahedrons.resize(num_elems);
    for (int i = 0; i < num_elems; i++) {
        tetrahedrons[i] = {elems[4*i+0], elems[4*i+1], elems[4*i+2], elems[4*i+3]};
    }
    fmt::print("Loading msh {}: nodes = {}, elems = {}\n", filename, num_nodes, num_elems);
}

}
