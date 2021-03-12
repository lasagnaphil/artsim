//
// Created by lasagnaphil on 21. 3. 12..
//

#include "artsim/art_with_soft_bodies.h"

#include "artsim/dynamics.h"

#include <iostream>
#include <filesystem>
#include <tinyxml2.h>
#include <glm/gtc/type_ptr.hpp>

using namespace glmx;
using namespace tinyxml2;
namespace fs = std::filesystem;

namespace artsim {

std::vector<double> split_to_double(const std::string& input, int num)
{
    std::vector<double> result;
    std::string::size_type sz = 0, nsz = 0;
    for(int i = 0; i < num; i++){
        result.push_back(std::stof(input.substr(sz), &nsz));
        sz += nsz;
    }
    return result;
}

glm::tvec1<real> string_to_vector1d(const std::string& input) {
    std::vector<double> v = split_to_double(input, 1);
    return glm::tvec1<real>(v[0]);
}

glm::tvec3<real> string_to_vector3d(const std::string& input) {
    std::vector<double> v = split_to_double(input, 3);
    return {v[0], v[1], v[2]};
}

glm::tvec4<real> string_to_vector4d(const std::string& input) {
    std::vector<double> v = split_to_double(input, 4);
    return {v[0], v[1], v[2], v[3]};
}

glm::tmat3x3<real> string_to_matrix3d(const std::string& input) {
    std::vector<double> v = split_to_double(input, 9);
    auto M = glm::transpose(glm::make_mat3x3(v.data()));
    return M;
}

bool load_from_xml(const char* filename, OUT ArticulatedBody& art) {
    std::unordered_map<std::string, ttransform<real>> T_global_body_map;
    std::unordered_map<std::string, ttransform<real>> T_global_joint_map;
    std::unordered_map<std::string, int> idx_map;

    T_global_body_map["none"] = ttransform<real>(IDENTITY);
    T_global_joint_map["none"] = ttransform<real>(IDENTITY);
    idx_map["none"] = -1;

    XMLDocument doc;
    if (doc.LoadFile(filename)) {
        std::cout << "Can't open file : " << filename << std::endl;
        return false;
    }

    XMLElement *art_elem = doc.FirstChildElement("articulation");
    std::string art_name = art_elem->Attribute("name");
    std::string art_xform_mode = art_elem->Attribute("xform_mode");
    if (art_xform_mode != "global") {
        std::cout << "Only xform_mode = global supported!" << std::endl;
        exit(EXIT_FAILURE);
    }

    int current_idx = 0;
    for(XMLElement* node = art_elem->FirstChildElement("node"); node != nullptr; node = node->NextSiblingElement("node"))
    {
        artsim::Joint joint;
        artsim::Link link;

        std::string name = node->Attribute("name");

        std::string parent_name = node->Attribute("parent");

        XMLElement* link_elem = node->FirstChildElement("link");
        std::string obj_file = "none";
        if(link_elem->Attribute("obj"))
            obj_file = link_elem->Attribute("obj");

        real mass = std::stod(link_elem->Attribute("mass"));

        std::string body_type = link_elem->Attribute("type");
        CollisionShape shape;
        if (body_type == "box") {
            glm::tvec3<real> size = string_to_vector3d(link_elem->Attribute("size"));
            shape = CollisionShape::make_box(size);
        }
        else if (body_type == "sphere") {
            double radius = std::stod(link_elem->Attribute("radius"));
            shape = CollisionShape::make_sphere(radius);
        }
        else if (body_type == "capsule") {
            double radius = std::stod(link_elem->Attribute("radius"));
            double height = std::stod(link_elem->Attribute("height"));
            printf("Capsule not supported!");
            return false;
        }

        real volume = shape.mass(real(1));
        real density = mass / volume;
        tsmat3x3<real> inertia = shape.inertia(density);

        ttransform<real> T_global_body;
        T_global_body.R = glmx::exp_mat(string_to_vector3d(link_elem->Attribute("rot")));
        T_global_body.v = string_to_vector3d(link_elem->Attribute("pos"));

        XMLElement* joint_elem = node->FirstChildElement("joint");
        std::string joint_type = joint_elem->Attribute("type");
        real joint_damping = joint_elem->DoubleAttribute("damping");

        ttransform<real> T_global_joint;
        T_global_joint.R = glmx::exp_mat(string_to_vector3d(joint_elem->Attribute("rot")));
        T_global_joint.v = string_to_vector3d(joint_elem->Attribute("pos"));

        T_global_body_map[name] = T_global_body;
        T_global_joint_map[name] = T_global_joint;

        ttransform<real> local_joint_pose;
        if (parent_name != "none") {
            local_joint_pose = T_global_joint / T_global_joint_map[parent_name];
        }
        else {
            local_joint_pose = ttransform<real>(IDENTITY);
        }
        ttransform<real> local_link_pose = T_global_body / T_global_joint;

        link = Link::create(inertia, mass, shape, local_joint_pose, local_link_pose, idx_map[parent_name], {});

        if(joint_type == "free")
        {
            // TODO: Should we also put kd on floating joints?
            joint = Joint::floating();
        }
        else if(joint_type == "ball")
        {
            joint = Joint::spherical(0, joint_damping);
        }
        else if(joint_type == "revolute")
        {
            glm::tvec3<real> axis = string_to_vector3d(joint_elem->Attribute("axis"));
            if (glm::epsilonEqual<real>(axis.x, 1.0, 1e-8)) {
                joint = Joint::revolute_x(0, joint_damping);
            }
            else if (glm::epsilonEqual<real>(axis.y, 1.0, 1e-8)) {
                joint = Joint::revolute_y(0, joint_damping);
            }
            else if (glm::epsilonEqual<real>(axis.z, 1.0, 1e-8)) {
                joint = Joint::revolute_z(0, joint_damping);
            }
            else {
                std::cout << "Only revolute joints with X, Y, or Z axis supported!" << std::endl;
                return false;
            }
        }

        art.add_link_and_joint(link, joint, name);
        idx_map[name] = current_idx;
        current_idx++;
    }

    art.setup();
    return true;
}


void ArtWithSoftBodies::load(const char* metadata) {

    fs::path metadata_path(metadata);
    fs::path folder = metadata_path.parent_path();

    XMLDocument doc;
    doc.LoadFile(metadata);
    auto root_el = doc.RootElement();

    auto articulation_el = root_el->FirstChildElement("articulation");
    auto art_file = folder / articulation_el->GetText();
    bool art_loaded = load_from_xml(art_file.c_str(), OUT art);
    if (!art_loaded) {
        exit(EXIT_FAILURE);
    }

    int sb_count = 0;
    for (XMLElement* sb_el = root_el->FirstChildElement("soft_body");
         sb_el != nullptr; sb_el = sb_el->NextSiblingElement("node"), sb_count++) {}
    soft_bodies.resize(sb_count);

    sb_start_idx.resize(sb_count+1);
    sb_start_idx[0] = 0;

    int sb_idx = 0;
    for (XMLElement* sb_el = root_el->FirstChildElement("soft_body"); sb_el != nullptr; sb_el = sb_el->NextSiblingElement("node")) {
        OBJFile soft_body_obj;
        fs::path soft_body_file = folder / sb_el->Attribute("file");
        if (soft_body_file.extension() == ".obj") {
            soft_body_obj.load_obj(soft_body_file.c_str());
        }
        else if (soft_body_file.extension() == ".msh") {
            soft_body_obj.load_msh(soft_body_file.c_str());
        }
        else {
            fprintf(stderr, "Invalid extension name for soft body mesh!\n");
            exit(EXIT_FAILURE);
        }

        SoftBodyProperties props;
        props.young_modulus = sb_el->DoubleAttribute("young_modulus");
        props.poisson_ratio = sb_el->DoubleAttribute("poisson_ratio");
        props.density = sb_el->DoubleAttribute("density");

        soft_bodies[sb_idx].load(soft_body_obj, props);

        sb_idx++;
        sb_start_idx[sb_idx] = sb_start_idx[sb_idx-1] + soft_bodies[sb_idx].vertices.size();
    }
}

}
