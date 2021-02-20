//
// Created by lasagnaphil on 21. 2. 17..
//

#include <artsim/artsim.h>
#include <artsim/dynamics.h>
#include <artsim/utils/xml.h>
#include <artsim/utils/pymesh/MshLoader.h>
#include <artsim/core/kdtree.h>
#include <tinyxml2.h>
#include <fmt/core.h>
#include <glm/gtx/hash.hpp>
#include <glm/gtx/string_cast.hpp>

#include <filesystem>
#include <unordered_set>

using namespace artsim;
using namespace glmx;
using namespace tinyxml2;
namespace fs = std::filesystem;

void dist_between_triangle_and_points(glm::rvec3 a, glm::rvec3 b, glm::rvec3 c,
                                      const glm::rvec3* points, int num_points, OUT float* dist) {

    rvec3 ba = b - a;
    rvec3 cb = c - b;
    rvec3 ac = a - c;
    rvec3 nor = cross( ba, ac );
    for (int i = 0; i < num_points; i++) {
        rvec3 p = points[i];
        rvec3 pa = p - a;
        rvec3 pb = p - b;
        rvec3 pc = p - c;
        dist[i] = sqrt(
                (sign(dot(cross(ba,nor),pa)) +
                 sign(dot(cross(cb,nor),pb)) +
                 sign(dot(cross(ac,nor),pc))<2.0)
                ?
                min( min(
                        length2(ba*clamp<real>(dot(ba,pa)/length2(ba),0,1)-pa),
                        length2(cb*clamp<real>(dot(cb,pb)/length2(cb),0,1)-pb) ),
                        length2(ac*clamp<real>(dot(ac,pc)/length2(ac),0,1)-pc) )
                :
                dot(nor,pa)*dot(nor,pa)/length2(nor) );
    }
}

int main(int argc, char** argv) {
    if (argc != 3) {
        printf("Usage: export_sb_art <articulation_file> <mesh_file>");
        exit(EXIT_FAILURE);
    }
    std::string art_file = argv[1];
    std::string mesh_file = argv[2];
    auto art_path = fs::path(art_file);
    auto mesh_path = fs::path(mesh_file);
    auto out_path = art_path.parent_path();
    auto metadata_path = out_path / "metadata.xml";

    std::vector<uint32_t> contact_indices;
    ArticulatedBody art = load_from_xml(art_file.c_str(), contact_indices);

    int num_links = art.get_num_joints();
    std::vector<ttransform<real>> link_trans(num_links), joint_trans(num_links);
    std::vector<real> rest_pose(art.get_num_pos_dofs(), 0);
    calc_transforms(art, rest_pose.data(), link_trans.data(), joint_trans.data());

    OBJFile mesh;
    mesh.load_obj(mesh_path.c_str());

    // discard normal data
    mesh.normals.clear();
    mesh.triangle_normals.clear();

    std::map<std::string, OBJFile> link_objs;

    for (int i = 0; i < num_links; i++) {
        auto name = art.names[i];
        auto& link = art.links[i];
        auto& shape = art.links[i].col_shape;

        OBJFile obj;

        int vidx = mesh.vertices.size();
        switch (shape.type) {
            case CollisionShape::Type::Box: {
                auto hs = 0.5 * shape.box.size;
                glm::vec3 b1 = link_trans[i].v - hs;
                glm::vec3 b2 = link_trans[i].v + hs;

                obj.vertices.emplace_back(b1.x, b1.y, b1.z);
                obj.vertices.emplace_back(b1.x, b1.y, b2.z);
                obj.vertices.emplace_back(b1.x, b2.y, b1.z);
                obj.vertices.emplace_back(b1.x, b2.y, b2.z);
                obj.vertices.emplace_back(b2.x, b1.y, b1.z);
                obj.vertices.emplace_back(b2.x, b1.y, b2.z);
                obj.vertices.emplace_back(b2.x, b2.y, b1.z);
                obj.vertices.emplace_back(b2.x, b2.y, b2.z);
                mesh.vertices.emplace_back(b1.x, b1.y, b1.z);
                mesh.vertices.emplace_back(b1.x, b1.y, b2.z);
                mesh.vertices.emplace_back(b1.x, b2.y, b1.z);
                mesh.vertices.emplace_back(b1.x, b2.y, b2.z);
                mesh.vertices.emplace_back(b2.x, b1.y, b1.z);
                mesh.vertices.emplace_back(b2.x, b1.y, b2.z);
                mesh.vertices.emplace_back(b2.x, b2.y, b1.z);
                mesh.vertices.emplace_back(b2.x, b2.y, b2.z);

                obj.triangle_vertices.emplace_back(1, 5, 7);
                obj.triangle_vertices.emplace_back(1, 7, 3);
                obj.triangle_vertices.emplace_back(1, 3, 4);
                obj.triangle_vertices.emplace_back(1, 4, 2);
                obj.triangle_vertices.emplace_back(3, 7, 8);
                obj.triangle_vertices.emplace_back(3, 8, 4);
                obj.triangle_vertices.emplace_back(5, 8, 7);
                obj.triangle_vertices.emplace_back(5, 6, 8);
                obj.triangle_vertices.emplace_back(1, 6, 5);
                obj.triangle_vertices.emplace_back(1, 2, 6);
                obj.triangle_vertices.emplace_back(2, 8, 6);
                obj.triangle_vertices.emplace_back(2, 4, 8);
                mesh.triangle_vertices.emplace_back(vidx + 1, vidx + 5, vidx + 7);
                mesh.triangle_vertices.emplace_back(vidx + 1, vidx + 7, vidx + 3);
                mesh.triangle_vertices.emplace_back(vidx + 1, vidx + 3, vidx + 4);
                mesh.triangle_vertices.emplace_back(vidx + 1, vidx + 4, vidx + 2);
                mesh.triangle_vertices.emplace_back(vidx + 3, vidx + 7, vidx + 8);
                mesh.triangle_vertices.emplace_back(vidx + 3, vidx + 8, vidx + 4);
                mesh.triangle_vertices.emplace_back(vidx + 5, vidx + 8, vidx + 7);
                mesh.triangle_vertices.emplace_back(vidx + 5, vidx + 6, vidx + 8);
                mesh.triangle_vertices.emplace_back(vidx + 1, vidx + 6, vidx + 5);
                mesh.triangle_vertices.emplace_back(vidx + 1, vidx + 2, vidx + 6);
                mesh.triangle_vertices.emplace_back(vidx + 2, vidx + 8, vidx + 6);
                mesh.triangle_vertices.emplace_back(vidx + 2, vidx + 4, vidx + 8);
            } break;
            case CollisionShape::Type::Sphere: {
                printf("Sphere shapes not supported yet\n");
                exit(EXIT_FAILURE);
            } break;
        }
        link_objs[name] = obj;
    }

    auto out_mesh_path = out_path / (mesh_path.stem().string() + "_carved.obj");
    mesh.save_obj(out_mesh_path.c_str());

    std::cout << "Carved out soft body mesh!" << std::endl;

    auto command = fmt::format("~/dev/TetWild/build/TetWild -l 0.05 {}", out_mesh_path.string());
    system(command.c_str());

    auto out_mesh_tet_path = out_mesh_path.parent_path() / (out_mesh_path.stem().string() + "_.msh");
    auto out_tet_mesh = PyMesh::MshLoader(out_mesh_tet_path.c_str());
    std::vector<glm::rvec3> tet_mesh_vertices;
    {
        auto& nodes = out_tet_mesh.get_nodes();
        tet_mesh_vertices.resize(nodes.size()/3);
        for (int i = 0; i < nodes.size()/3; i++) {
            tet_mesh_vertices.emplace_back(nodes[3*i+0], nodes[3*i+1], nodes[3*i+2]);
        }
    }

    XMLDocument doc;
    auto root_el = doc.NewElement("metadata");
    doc.InsertEndChild(root_el);

    auto art_el = doc.NewElement("articulation");
    art_el->SetText(art_path.filename().c_str());
    root_el->InsertEndChild(art_el);

    auto soft_body_mesh_el = doc.NewElement("soft_body_mesh");

    soft_body_mesh_el->SetText(out_mesh_tet_path.filename().c_str());
    root_el->InsertEndChild(soft_body_mesh_el);

    std::unordered_map<glm::vec3, int> vertex_map;
    for (int i = 0; i < tet_mesh_vertices.size(); i++) {
        if (vertex_map.find(tet_mesh_vertices[i]) == vertex_map.end()) {
            vertex_map.insert({tet_mesh_vertices[i], i});
        }
    }

    const real threshold = 1e-4;

    std::unordered_map<std::string, std::vector<int>> linked_vertices;
    for (int i = 0; i < num_links; i++) {
        linked_vertices[art.names[i]] = {};
    }

    for (int i = 0; i < num_links; i++) {
        std::unordered_set<glm::rvec3> found_vertices;

        auto& name = art.names[i];
        auto& link_obj = link_objs[name];
        for (const glm::ivec3& tri : link_obj.triangle_vertices) {
            std::vector<float> dist(tet_mesh_vertices.size());
            auto v0 = link_obj.vertices[tri[0]];
            auto v1 = link_obj.vertices[tri[1]];
            auto v2 = link_obj.vertices[tri[2]];
            dist_between_triangle_and_points(v0, v1, v2, tet_mesh_vertices.data(), tet_mesh_vertices.size(),
                                             OUT dist.data());
            for (int j = 0; j < tet_mesh_vertices.size(); j++) {
                if (dist[j] < threshold && found_vertices.find(tet_mesh_vertices[j]) == found_vertices.end()) {
                    found_vertices.insert(tet_mesh_vertices[j]);
                }
            }
        }

        fmt::print("For link {}: \n", name);
        for (auto& v : found_vertices) {
            fmt::print("{}\n", glm::to_string(v));
            linked_vertices[name].push_back(vertex_map[v]);
        }
    }

    auto constraints_el = doc.NewElement("constraints");
    for (int i = 0; i < art.get_num_joints(); i++) {
        auto& name = art.names[i];
        auto& vertices = linked_vertices[name];
        auto link_el = doc.NewElement("link");
        link_el->SetAttribute("name", name.c_str());
        constraints_el->InsertEndChild(link_el);
        auto vertices_el = doc.NewElement("vertices");
        std::string vertices_text;
        for (auto& v : vertices) {
            vertices_text += std::to_string(v);
            vertices_text += " ";
        }
        vertices_el->SetText(vertices_text.c_str());
        link_el->InsertEndChild(vertices_el);
    }
    root_el->InsertFirstChild(constraints_el);

    doc.SaveFile(metadata_path.c_str());

    return 0;
}