//
// Created by lasagnaphil on 21. 2. 17..
//

#include <artsim/artsim.h>
#include <artsim/dynamics.h>
#include <artsim/utils/xml.h>
#include <tinyxml2.h>

#include <filesystem>

using namespace artsim;
using namespace glmx;
using namespace tinyxml2;
namespace fs = std::filesystem;

int main(int argc, char** argv) {
    if (argc != 3) {
        printf("Usage: export_sb_art <obj_file> <articulation_file>");
        exit(EXIT_FAILURE);
    }
    std::string obj_file = argv[1];
    std::string art_file = argv[2];
    auto obj_path = fs::path(obj_file);
    auto art_path = fs::path(art_file);
    auto metadata_path = obj_path.parent_path() / "metadata.xml";

    OBJFile obj;
    obj.load(obj_file.c_str());

    std::vector<uint32_t> contact_indices;
    ArticulatedBody art = load_from_xml(art_file.c_str(), contact_indices);

    std::unordered_map<std::string, std::pair<int, int>> linked_vertices_range;
    std::unordered_map<std::string, std::pair<int, int>> linked_normals_range;

    int num_links = art.get_num_joints();
    std::vector<ttransform<real>> link_trans(num_links), joint_trans(num_links);
    std::vector<real> rest_pose(art.get_num_pos_dofs(), 0);
    calc_transforms(art, rest_pose.data(), link_trans.data(), joint_trans.data());

    for (int i = 0; i < num_links; i++) {
        auto name = art.names[i];
        auto& link = art.links[i];
        auto& shape = art.links[i].col_shape;
        switch (shape.type) {
            case CollisionShape::Type::Box: {
                auto hs = 0.5 * shape.box.size;
                glm::vec3 b1 = link_trans[i].v - hs;
                glm::vec3 b2 = link_trans[i].v + hs;
                int vidx = obj.vertices.size();
                int nidx = obj.normals.size();

                obj.vertices.emplace_back(b1.x, b1.y, b1.z);
                obj.vertices.emplace_back(b1.x, b1.y, b2.z);
                obj.vertices.emplace_back(b1.x, b2.y, b1.z);
                obj.vertices.emplace_back(b1.x, b2.y, b2.z);
                obj.vertices.emplace_back(b2.x, b1.y, b1.z);
                obj.vertices.emplace_back(b2.x, b1.y, b2.z);
                obj.vertices.emplace_back(b2.x, b2.y, b1.z);
                obj.vertices.emplace_back(b2.x, b2.y, b2.z);

                obj.normals.emplace_back( 0,  0, -1);
                obj.normals.emplace_back( 0,  0,  1);
                obj.normals.emplace_back( 0, -1,  0);
                obj.normals.emplace_back( 0,  1,  0);
                obj.normals.emplace_back(-1,  0,  0);
                obj.normals.emplace_back( 1,  0,  0);

                obj.triangle_vertices.emplace_back(vidx + 1, vidx + 7, vidx + 5);
                obj.triangle_vertices.emplace_back(vidx + 1, vidx + 3, vidx + 7);
                obj.triangle_vertices.emplace_back(vidx + 1, vidx + 4, vidx + 3);
                obj.triangle_vertices.emplace_back(vidx + 1, vidx + 2, vidx + 4);
                obj.triangle_vertices.emplace_back(vidx + 3, vidx + 8, vidx + 7);
                obj.triangle_vertices.emplace_back(vidx + 3, vidx + 4, vidx + 8);
                obj.triangle_vertices.emplace_back(vidx + 5, vidx + 7, vidx + 8);
                obj.triangle_vertices.emplace_back(vidx + 5, vidx + 8, vidx + 6);
                obj.triangle_vertices.emplace_back(vidx + 1, vidx + 5, vidx + 6);
                obj.triangle_vertices.emplace_back(vidx + 1, vidx + 6, vidx + 2);
                obj.triangle_vertices.emplace_back(vidx + 2, vidx + 6, vidx + 8);
                obj.triangle_vertices.emplace_back(vidx + 2, vidx + 8, vidx + 4);

                obj.triangle_normals.emplace_back(nidx + 2, nidx + 2, nidx + 2);
                obj.triangle_normals.emplace_back(nidx + 2, nidx + 2, nidx + 2);
                obj.triangle_normals.emplace_back(nidx + 6, nidx + 6, nidx + 6);
                obj.triangle_normals.emplace_back(nidx + 6, nidx + 6, nidx + 6);
                obj.triangle_normals.emplace_back(nidx + 3, nidx + 3, nidx + 3);
                obj.triangle_normals.emplace_back(nidx + 3, nidx + 3, nidx + 3);
                obj.triangle_normals.emplace_back(nidx + 5, nidx + 5, nidx + 5);
                obj.triangle_normals.emplace_back(nidx + 5, nidx + 5, nidx + 5);
                obj.triangle_normals.emplace_back(nidx + 4, nidx + 4, nidx + 4);
                obj.triangle_normals.emplace_back(nidx + 4, nidx + 4, nidx + 4);
                obj.triangle_normals.emplace_back(nidx + 1, nidx + 1, nidx + 1);
                obj.triangle_normals.emplace_back(nidx + 1, nidx + 1, nidx + 1);

                linked_vertices_range[name] = {vidx, obj.vertices.size()};
                linked_normals_range[name] = {nidx, obj.normals.size()};
            } break;
            case CollisionShape::Type::Sphere: {
                printf("Sphere shapes not supported yet\n");
                exit(EXIT_FAILURE);
            } break;
        }
    }

    auto obj_carved_path = obj_path.parent_path() / (obj_path.stem().string() + "_carved.obj");
    obj.save(obj_carved_path.c_str());

    XMLDocument doc;
    auto root_el = doc.NewElement("metadata");
    doc.InsertEndChild(root_el);

    auto art_el = doc.NewElement("articulation");
    art_el->SetText(art_path.filename().c_str());
    root_el->InsertEndChild(art_el);

    auto soft_body_mesh_el = doc.NewElement("soft_body_mesh");
    soft_body_mesh_el->SetText(obj_carved_path.filename().c_str());
    root_el->InsertEndChild(soft_body_mesh_el);

    auto constraints_el = doc.NewElement("constraints");
    for (int i = 0; i < art.get_num_joints(); i++) {
        auto& name = art.names[i];
        if (linked_vertices_range.find(name) == linked_vertices_range.end()) {
            continue;
        }
        auto& vertices = linked_vertices_range[name];
        auto& normals = linked_normals_range[name];
        auto link_el = doc.NewElement("link");
        link_el->SetAttribute("name", name.c_str());
        constraints_el->InsertEndChild(link_el);
        auto vertices_el = doc.NewElement("vertices");
        vertices_el->SetAttribute("start", vertices.first);
        vertices_el->SetAttribute("end", vertices.second);
        link_el->InsertEndChild(vertices_el);
        auto indices_el = doc.NewElement("normals");
        indices_el->SetAttribute("start", vertices.first);
        indices_el->SetAttribute("end", vertices.second);
        link_el->InsertEndChild(indices_el);
    }
    root_el->InsertFirstChild(constraints_el);

    doc.SaveFile(metadata_path.c_str());

    return 0;
}