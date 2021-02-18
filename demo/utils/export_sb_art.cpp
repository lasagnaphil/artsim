//
// Created by lasagnaphil on 21. 2. 17..
//

#include <artsim/artsim.h>
#include <artsim/utils/xml.h>

using namespace artsim;

/*
v -2 -1 -1
v -2 -1 1
v -2 1 -1
v -2 1 1
v 2 -1 -1
v 2 -1 1
v 2 1 -1
v 2 1 1

vn  0.0  0.0  1.0
vn  0.0  0.0 -1.0
vn  0.0  1.0  0.0
vn  0.0 -1.0  0.0
vn  1.0  0.0  0.0
vn -1.0  0.0  0.0

f  1//2  7//2  5//2
f  1//2  3//2  7//2
f  1//6  4//6  3//6
f  1//6  2//6  4//6
f  3//3  8//3  7//3
f  3//3  4//3  8//3
f  5//5  7//5  8//5
f  5//5  8//5  6//5
f  1//4  5//4  6//4
f  1//4  6//4  2//4
f  2//1  6//1  8//1
f  2//1  8//1  4//1
 */

int main(int argc, char** argv) {
    if (argc != 3) {
        printf("Usage: export_sb_art <obj_file> <articulation_file>");
        exit(EXIT_FAILURE);
    }
    std::string obj_file = argv[1];
    std::string art_file = argv[2];

    OBJFile obj;
    obj.load(obj_file.c_str());

    std::vector<uint32_t> contact_indices;
    ArticulatedBody art = load_from_xml(obj_file.c_str(), contact_indices);

    std::unordered_map<std::string, std::pair<int, int>> linked_vertices_range;
    std::unordered_map<std::string, std::pair<int, int>> linked_normals_range;

    for (int i = 0; i < art.get_num_joints(); i++) {
        auto name = art.names[i];
        auto& link = art.links[i];
        auto& shape = art.links[i].col_shape;
        switch (shape.type) {
            case CollisionShape::Type::Box: {
                auto hs = 0.5 * shape.box.size;
                int vidx = obj.vertices.size();
                int nidx = obj.normals.size();

                obj.vertices.emplace_back(-hs.x, -hs.y, -hs.z);
                obj.vertices.emplace_back(-hs.x, -hs.y,  hs.z);
                obj.vertices.emplace_back(-hs.x,  hs.y, -hs.z);
                obj.vertices.emplace_back(-hs.x,  hs.y,  hs.z);
                obj.vertices.emplace_back( hs.x, -hs.y, -hs.z);
                obj.vertices.emplace_back( hs.x, -hs.y,  hs.z);
                obj.vertices.emplace_back( hs.x,  hs.y, -hs.z);
                obj.vertices.emplace_back( hs.x,  hs.y,  hs.z);

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
    return 0;
}