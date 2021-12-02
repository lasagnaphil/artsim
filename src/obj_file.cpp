//
// Created by lasagnaphil on 2/16/21.
//

#include "artsim/obj_file.h"
#include <mshio/mshio.h>

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
        if (str.empty()) continue;
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
        ofs << "v " << v.x << " " << v.y << " " << v.z << '\n';
    }
    for (auto& vn : normals) {
        ofs << "vn " << vn.x << " " << vn.y << " " << vn.z << '\n';
    }
    for (auto& vt : uvs) {
        ofs << "vt " << vt.x << " " << vt.y << '\n';
    }
    if (!triangle_vertices.empty() && !triangle_uvs.empty() && !triangle_normals.empty() ) {
        for (int i = 0; i < triangle_vertices.size(); i++) {
            auto fi = triangle_vertices[i]+1;
            auto ft = triangle_uvs[i]+1;
            auto fn = triangle_normals[i]+1;
            ofs << "f " << fi[0] << "/" << ft[0] << "/" << fn[0] << " "
                        << fi[1] << "/" << ft[1] << "/" << fn[1] << " "
                        << fi[2] << "/" << ft[2] << "/" << fn[2] << '\n';
        }
    }
    else if (!triangle_vertices.empty() && !triangle_normals.empty()) {
        for (int i = 0; i < triangle_vertices.size(); i++) {
            auto fi = triangle_vertices[i]+1;
            auto fn = triangle_normals[i]+1;
            ofs << "f " << fi[0] << "//" << fn[0] << " "
                        << fi[1] << "//" << fn[1] << " "
                        << fi[2] << "//" << fn[2] << '\n';
        }
    }
    else if (!triangle_vertices.empty()) {
        for (int i = 0; i < triangle_vertices.size(); i++) {
            auto fi = triangle_vertices[i]+1;
            ofs << "f " << fi[0] << " " << fi[1] << " " << fi[2] << '\n';
        }
    }
    for (auto& t : tetrahedrons) {
        ofs << "t " << t[0] << " " << t[1] << " " << t[2] << " " << t[3] << '\n';
    }
}

void OBJFile::load_msh(const char* filename) {
    auto spec = mshio::load_msh(filename);
    int num_nodes = spec.nodes.num_nodes;
    int num_elems = spec.elements.num_elements;
    vertices.resize(num_nodes);
    auto& vert_data = spec.nodes.entity_blocks[0].data;
    for (int i = 0; i < num_nodes; i++) {
        vertices[i] = {vert_data[3*i+0], vert_data[3*i+1], vert_data[3*i+2]};
    }
    tetrahedrons.resize(num_elems);
    auto& elem_data = spec.elements.entity_blocks[0].data;
    for (int i = 0; i < num_elems; i++) {
        tetrahedrons[i] = {elem_data[4*i+0], elem_data[4*i+1], elem_data[4*i+2], elem_data[4*i+3]};
    }
    printf("Loading msh %s: nodes = %d, tets = %d\n", filename, num_nodes, num_elems);
}

void OBJFile::save_msh(const char* filename) {
    mshio::MshSpec spec;
    spec.mesh_format.file_type = 1;
    spec.nodes.num_entity_blocks = 1;
    spec.nodes.num_nodes = vertices.size();
    spec.nodes.min_node_tag = 0;
    spec.nodes.max_node_tag = vertices.size()-1;
    spec.nodes.entity_blocks.resize(1);
    spec.nodes.entity_blocks[0].entity_dim = 3;
    spec.nodes.entity_blocks[0].num_nodes_in_block = vertices.size();
    auto& vert_data = spec.nodes.entity_blocks[0].data;
    auto& vert_indices = spec.nodes.entity_blocks[0].tags;
    vert_data.resize(3*vertices.size());
    vert_indices.resize(vertices.size());
    for (int i = 0; i < vertices.size(); i++) {
        vert_indices[i] = i;
        vert_data[3*i+0] = vertices[i][0];
        vert_data[3*i+1] = vertices[i][1];
        vert_data[3*i+2] = vertices[i][2];
    }
    spec.elements.num_entity_blocks = 1;
    spec.elements.num_elements = tetrahedrons.size();
    spec.elements.min_element_tag = 0;
    spec.elements.max_element_tag = tetrahedrons.size()-1;
    spec.elements.entity_blocks.resize(1);
    spec.elements.entity_blocks[0].entity_dim = 3;
    spec.elements.entity_blocks[0].element_type = 4; // 4-node tetrahedron
    spec.elements.entity_blocks[0].num_elements_in_block = tetrahedrons.size();
    auto& elem_data = spec.elements.entity_blocks[0].data;
    elem_data.resize(5*tetrahedrons.size());
    for (int i = 0; i < tetrahedrons.size(); i++) {
        elem_data[5*i+0] = i;
        elem_data[5*i+1] = tetrahedrons[i][0];
        elem_data[5*i+2] = tetrahedrons[i][1];
        elem_data[5*i+3] = tetrahedrons[i][2];
        elem_data[5*i+4] = tetrahedrons[i][3];
    }
    mshio::validate_spec(spec);
    mshio::save_msh(filename, spec);
}

}
