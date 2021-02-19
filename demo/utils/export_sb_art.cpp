//
// Created by lasagnaphil on 21. 2. 17..
//

#include <artsim/artsim.h>
#include <artsim/dynamics.h>
#include <artsim/utils/xml.h>
#include <artsim/utils/pymesh/MshLoader.h>
#include <tinyxml2.h>
#include <fmt/core.h>

#include <filesystem>

using namespace artsim;
using namespace glmx;
using namespace tinyxml2;
namespace fs = std::filesystem;

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
    mesh.load(mesh_path.c_str());

    // discard normal data
    mesh.normals.clear();
    mesh.triangle_normals.clear();

    std::unordered_map<std::string, std::pair<int, int>> linked_vertices_range;
    std::unordered_map<std::string, std::pair<int, int>> linked_faces_range;

    for (int i = 0; i < num_links; i++) {
        auto name = art.names[i];
        auto& link = art.links[i];
        auto& shape = art.links[i].col_shape;

        int vidx = mesh.vertices.size();
        int tidx = mesh.triangle_vertices.size();
        switch (shape.type) {
            case CollisionShape::Type::Box: {
                auto hs = 0.5 * shape.box.size;
                glm::vec3 b1 = link_trans[i].v - hs;
                glm::vec3 b2 = link_trans[i].v + hs;

                mesh.vertices.emplace_back(b1.x, b1.y, b1.z);
                mesh.vertices.emplace_back(b1.x, b1.y, b2.z);
                mesh.vertices.emplace_back(b1.x, b2.y, b1.z);
                mesh.vertices.emplace_back(b1.x, b2.y, b2.z);
                mesh.vertices.emplace_back(b2.x, b1.y, b1.z);
                mesh.vertices.emplace_back(b2.x, b1.y, b2.z);
                mesh.vertices.emplace_back(b2.x, b2.y, b1.z);
                mesh.vertices.emplace_back(b2.x, b2.y, b2.z);

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

        linked_vertices_range[name] = {vidx, mesh.vertices.size()};
        linked_faces_range[name] = {tidx, mesh.triangle_vertices.size()};
    }

    auto out_mesh_path = out_path / (mesh_path.stem().string() + "_carved.obj");
    mesh.save(out_mesh_path.c_str());

    std::cout << "Carved out soft body mesh!" << std::endl;

    auto command = fmt::format("~/dev/TetWild/build/TetWild -l 0.05 {}", out_mesh_path.string());
    system(command.c_str());

    XMLDocument doc;
    auto root_el = doc.NewElement("metadata");
    doc.InsertEndChild(root_el);

    auto art_el = doc.NewElement("articulation");
    art_el->SetText(art_path.filename().c_str());
    root_el->InsertEndChild(art_el);

    auto soft_body_mesh_el = doc.NewElement("soft_body_mesh");
    auto out_mesh_tet_path = out_mesh_path.parent_path() / (out_mesh_path.stem().string() + "_.msh");
    soft_body_mesh_el->SetText(out_mesh_tet_path.filename().c_str());
    root_el->InsertEndChild(soft_body_mesh_el);

    auto constraints_el = doc.NewElement("constraints");
    for (int i = 0; i < art.get_num_joints(); i++) {
        auto& name = art.names[i];
        if (linked_vertices_range.find(name) == linked_vertices_range.end()) {
            continue;
        }
        auto& vertices = linked_vertices_range[name];
        auto& normals = linked_faces_range[name];
        auto link_el = doc.NewElement("link");
        link_el->SetAttribute("name", name.c_str());
        constraints_el->InsertEndChild(link_el);
        auto vertices_el = doc.NewElement("vertices");
        vertices_el->SetAttribute("start", vertices.first);
        vertices_el->SetAttribute("end", vertices.second);
        link_el->InsertEndChild(vertices_el);
        auto faces_el = doc.NewElement("faces");
        faces_el ->SetAttribute("start", vertices.first);
        faces_el ->SetAttribute("end", vertices.second);
        link_el->InsertEndChild(faces_el);
    }
    root_el->InsertFirstChild(constraints_el);

    doc.SaveFile(metadata_path.c_str());

    return 0;
}