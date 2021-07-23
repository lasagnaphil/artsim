//
// Created by lasagnaphil on 21. 2. 8..
//

#include "artsim/soft_body.h"
#include "artsim/math/svd.h"
#include "artsim/math/fastsvd.h"

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>
#include <unordered_map>
#include <unordered_set>
#include <deque>
#include <glm/gtx/hash.hpp>
#include <glm/gtx/string_cast.hpp>
#include <fmt/core.h>

#include <Tracy.hpp>

using namespace Eigen;
using MatrixXr = Matrix<artsim::real, Dynamic, Dynamic>;
using VectorXr = Matrix<artsim::real, Dynamic, 1>;

namespace artsim {

std::pair<glm::ivec3, bool> reorder_tri_indices(glm::ivec3 tri) {
    bool flipped = false;
    if (tri[1] > tri[2]) {
        flipped = !flipped;
        std::swap(tri[1], tri[2]);
    }
    if (tri[0] > tri[1]) {
        flipped = !flipped;
        std::swap(tri[0], tri[1]);
    }
    if (tri[1] > tri[2]) {
        flipped = !flipped;
        std::swap(tri[1], tri[2]);
    }
    return {tri, flipped};
}

void gen_surface_triangles_from_tet_mesh(const std::vector<glm::ivec4>& tetrahedrons,
                                         OUT std::vector<glm::ivec3>& triangles) {
    std::unordered_map<glm::ivec3, std::pair<int, bool>> tri_overlaps;
    auto insert_triangle = [&](glm::ivec3 tri) {
        auto [tri_p, flipped] = reorder_tri_indices(tri);
        auto it = tri_overlaps.find(tri_p);
        if (it == tri_overlaps.end()) {
            tri_overlaps.insert({tri_p, {1, flipped}});
        }
        else {
            it->second.first++;
        }
    };
    for (auto& tet : tetrahedrons) {
        insert_triangle({tet[0], tet[2], tet[1]});
        insert_triangle({tet[0], tet[1], tet[3]});
        insert_triangle({tet[0], tet[3], tet[2]});
        insert_triangle({tet[1], tet[2], tet[3]});
    }
    for (auto [tri, p] : tri_overlaps) {
        auto [count, flipped] = p;
        if (count == 1) {
            if (flipped) {
                glm::ivec3 tri_p(tri[1], tri[0], tri[2]);
                triangles.push_back(tri_p);
            }
            else {
                triangles.push_back(tri);
            }
        }
    }
}

void gen_edges_from_surface_tri_mesh(const std::vector<glm::ivec3>& triangles,
                                     OUT std::vector<glm::ivec2>& edges) {
    std::unordered_set<glm::ivec2> edge_set;
    for (auto& tri : triangles) {
        edge_set.insert({tri[0], tri[1]});
        edge_set.insert({tri[1], tri[2]});
        edge_set.insert({tri[2], tri[0]});
    }
    edges.insert(edges.end(), edge_set.begin(), edge_set.end());
}

void gen_verts_from_surface_tri_mesh(const std::vector<glm::ivec3>& triangles,
                                     OUT std::vector<int>& vertices) {
    std::unordered_set<int> vert_set;
    for (auto& tri : triangles) {
        vert_set.insert(tri[0]);
        vert_set.insert(tri[1]);
        vert_set.insert(tri[2]);
    }
    vertices.insert(vertices.end(), vert_set.begin(), vert_set.end());
}

void gen_edges_from_tet_mesh(const std::vector<glm::ivec4>& tets,
                             OUT std::vector<glm::ivec2>& edges) {
    std::unordered_set<glm::ivec2> edge_set;
    for (auto& tet : tets) {
        edge_set.insert({tet[0], tet[1]});
        edge_set.insert({tet[1], tet[2]});
        edge_set.insert({tet[2], tet[3]});
        edge_set.insert({tet[3], tet[0]});
    }
    edges.insert(edges.end(), edge_set.begin(), edge_set.end());
}

void SoftBody::load(const TetMesh& mesh) {
    verts = mesh.vertices;
    tets = mesh.tetrahedrons;
    gen_surface_triangles_from_tet_mesh(tets, OUT surface_triangles);
    gen_edges_from_surface_tri_mesh(surface_triangles, OUT surface_edges);
    gen_verts_from_surface_tri_mesh(surface_triangles, OUT surface_verts);

    B_m.resize(tets.size());
    W.resize(tets.size());
    D.resize(tets.size());
    for (int i = 0; i < tets.size(); i++) {
        auto& V = verts;
        glm::ivec4& tet = tets[i];
        glm::tmat3x3<real> D_m(V[tet[0]] - V[tet[3]], V[tet[1]] - V[tet[3]], V[tet[2]] - V[tet[3]]);
        W[i] = glm::determinant(D_m) / 6.0;
        if (W[i] < 0) {
            W[i] = -W[i];
            std::swap(tet[2], tet[3]);
            D_m = glm::tmat3x3<real>(V[tet[0]] - V[tet[3]], V[tet[1]] - V[tet[3]], V[tet[2]] - V[tet[3]]);
        }
        B_m[i] = glm::inverse(D_m);
        glm::tmat3x3<real> D_i = glm::transpose(B_m[i]);
        D[i][0] = D_i[0];
        D[i][1] = D_i[1];
        D[i][2] = D_i[2];
        D[i][3] = -D_i[0] - D_i[1] - D_i[2];
    }
}

void SoftBody::load(const PyMesh::MshLoader& msh) {
    auto& nodes = msh.get_nodes();
    auto& elems = msh.get_elements();
    int num_nodes = nodes.rows() / 3;
    int num_elems = elems.rows() / 4;
    verts.resize(num_nodes);
    for (int i = 0; i < num_nodes; i++) {
        verts[i] = {nodes[3 * i + 0], nodes[3 * i + 1], nodes[3 * i + 2]};
    }
    tets.resize(num_elems);
    for (int i = 0; i < num_elems; i++) {
        tets[i] = {elems[4 * i + 0], elems[4 * i + 1], elems[4 * i + 2], elems[4 * i + 3]};
    }
    gen_surface_triangles_from_tet_mesh(tets, OUT surface_triangles);
}

void SoftBody::build_mass(real density, real dt) {
    ZoneScoped
    int num_vertices = verts.size();
    int num_tets = tets.size();
    A.resize(3 * num_vertices, 3 * num_vertices);

    std::vector<Eigen::Triplet<real>> A_triplets;
    std::unordered_map<glm::ivec2, real> A_triplets_map;
    for (int i = 0; i < num_tets; i++) {
        real m = density * W[i] / real(20.0) / (dt * dt);
        glm::ivec4 tet = tets[i];
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                for (int l = 0; l < 3; l++) {
                    auto idx = glm::ivec2(3 * tet[j] + l, 3 * tet[k] + l);
                    auto it = A_triplets_map.find(idx);
                    if (it != A_triplets_map.end()) {
                        it->second += j == k ? 2.0 * m : m;
                    } else {
                        A_triplets_map.insert({idx, j == k ? 2.0 * m : m});
                    }
                }
            }
        }
    }
    A_triplets.reserve(A_triplets_map.size());
    for (auto&[k, v] : A_triplets_map) {
        A_triplets.emplace_back(k[0], k[1], v);
    }

    A.setFromTriplets(A_triplets.begin(), A_triplets.end());
    M = A;

    // Factorize mass matrix (not going to change over the course of simulation, so better do it now)
    M_LDLt.analyzePattern(M);
    M_LDLt.factorize(M);
}

void SoftBody::clear_mass() {
    A.setZero();
    M.setZero();
}

}
