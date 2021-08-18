//
// Created by lasagnaphil on 21. 2. 17..
//

#include "artsim/utils/xml.h"

#include <iostream>
#include <tinyxml2.h>
#include <glm/gtc/type_ptr.hpp>
#include <filesystem>

using namespace tinyxml2;
using namespace glmx;
namespace fs = std::filesystem;

static std::vector<double> split_to_double(const std::string& input, int num)
{
    std::vector<double> result;
    std::string::size_type sz = 0, nsz = 0;
    for(int i = 0; i < num; i++){
        result.push_back(std::stof(input.substr(sz), &nsz));
        sz += nsz;
    }
    return result;
}

static glm::rvec3 string_to_vector3d(const std::string& input) {
    std::vector<double> v = split_to_double(input, 3);
    return {v[0], v[1], v[2]};
}

static glm::rvec4 string_to_vector4d(const std::string& input) {
    std::vector<double> v = split_to_double(input, 4);
    return {v[0], v[1], v[2], v[3]};
}

static glm::rmat3 string_to_matrix3d(const std::string& input) {
    std::vector<double> v = split_to_double(input, 9);
    auto M = glm::transpose(glm::make_mat3x3(v.data()));
    return M;
}

static std::string to_string(glm::vec3 v) {
    char s[30];
    sprintf(s, "%.6g %.6g %.6g", v[0], v[1], v[2]);
    return std::string(s);
}

static std::string to_string(artsim::real v) {
    char s[12];
    sprintf(s, "%.6g", v);
    return s;
}

namespace artsim {

bool load_from_xml_legacy(tinyxml2::XMLElement* root_el, OUT ArticulatedBodySpec& art) {
    std::unordered_map<std::string, ttransform<real>> T_global_body_map;
    std::unordered_map<std::string, ttransform<real>> T_global_joint_map;
    std::unordered_map<std::string, int> idx_map;

    T_global_body_map["None"] = ttransform<real>(IDENTITY);
    T_global_joint_map["None"] = ttransform<real>(IDENTITY);
    idx_map["None"] = -1;

    XMLElement *skeleton_elem = root_el;
    std::string skel_name = skeleton_elem->Attribute("name");

    int current_idx = 0;
    for(XMLElement* node = skeleton_elem->FirstChildElement("Node"); node != nullptr; node = node->NextSiblingElement("Node"))
    {
        artsim::Joint joint;
        artsim::Link link;

        std::string name = node->Attribute("name");

        std::string parent_name = node->Attribute("parent");

        XMLElement* body_elem = node->FirstChildElement("Body");
        std::string obj_file = "None";
        if(body_elem->Attribute("obj"))
            obj_file = body_elem->Attribute("obj");

        real mass = std::stod(body_elem->Attribute("mass"));

        std::string body_type = body_elem->Attribute("type");
        CollisionShape shape;
        if (body_type == "Box") {
            glm::rvec3 size = string_to_vector3d(body_elem->Attribute("size"));
            shape = CollisionShape::make_box(size);
        }
        else if (body_type == "Sphere") {
            double radius = std::stod(body_elem->Attribute("radius"));
            shape = CollisionShape::make_sphere(radius);
        }
        else if (body_type == "Capsule") {
            double radius = std::stod(body_elem->Attribute("radius"));
            double height = std::stod(body_elem->Attribute("height"));
            printf("Capsule not supported!\n");
            exit(EXIT_FAILURE);
        }

        real volume = shape.mass(real(1));
        real density = mass / volume;
        tsmat3x3<real> inertia = shape.inertia(density);

        bool contact = false;
        if(body_elem->Attribute("contact") != nullptr){
            std::string c = body_elem->Attribute("contact");
            if(c == "On") contact = true;
        }

        ttransform<real> T_global_body;
        T_global_body.R = string_to_matrix3d(body_elem->FirstChildElement("Transformation")->Attribute("linear"));
        T_global_body.v = string_to_vector3d(body_elem->FirstChildElement("Transformation")->Attribute("translation"));

        XMLElement* joint_elem = node->FirstChildElement("Joint");
        std::string joint_type = joint_elem->Attribute("type");
        ttransform<real> T_global_joint;
        T_global_joint.R = string_to_matrix3d(joint_elem->FirstChildElement("Transformation")->Attribute("linear"));
        T_global_joint.v = string_to_vector3d(joint_elem->FirstChildElement("Transformation")->Attribute("translation"));

        T_global_body_map[name] = T_global_body;
        T_global_joint_map[name] = T_global_joint;

        ttransform<real> local_joint_pose = T_global_joint / T_global_joint_map[parent_name];
        ttransform<real> local_link_pose = T_global_body / T_global_joint;

        link = Link::create(shape, mass, inertia, local_joint_pose, local_link_pose, idx_map[parent_name], {});

        real kp = 0.0;
        real kd = 0.0;
        auto kp_str = joint_elem->Attribute("kp");
        if (kp_str) {
            kp = std::stod(kp_str);
        }
        auto kd_str = joint_elem->Attribute("kd");
        if (kd_str) {
            kd = std::stod(kd_str);
        }
        if(joint_type == "Free")
        {
            // TODO: Should we also put kd on floating joints?
            joint = Joint::floating();
        }
        else if(joint_type == "Ball")
        {
            joint = Joint::spherical(kp, kd);
        }
        else if(joint_type == "Revolute")
        {
            glm::rvec3 axis = string_to_vector3d(joint_elem->Attribute("axis"));
            if (glm::epsilonEqual<real>(axis.x, 1.0, 1e-8)) {
                joint = Joint::revolute_x(kp, kd);
            }
            else if (glm::epsilonEqual<real>(axis.y, 1.0, 1e-8)) {
                joint = Joint::revolute_y(kp, kd);
            }
            else if (glm::epsilonEqual<real>(axis.z, 1.0, 1e-8)) {
                joint = Joint::revolute_z(kp, kd);
            }
        }

        art.add_link_and_joint(link, joint, name);
        idx_map[name] = current_idx;
        current_idx++;
    }

    art.build();
    return true;
}

XMLError load_from_xml_legacy(const char* filename, OUT ArticulatedBodySpec& art) {
    XMLDocument doc;
    XMLError err = doc.LoadFile(filename);
    if (err != XMLError::XML_SUCCESS) {
        std::cout << "Can't open file : " << filename << std::endl;
        return err;
    }
    // TODO: better error checking
    if (load_from_xml_legacy(doc.RootElement(), art)) {
        return err;
    }
    else {
        return err;
    }
}

bool load_from_xml(tinyxml2::XMLElement* art_elem, const char* current_dir, OUT ArticulatedBodySpec& spec) {
    std::unordered_map<std::string, ttransform<real>> T_global_body_map;
    std::unordered_map<std::string, ttransform<real>> T_global_joint_map;
    std::unordered_map<std::string, int> idx_map;

    T_global_body_map["none"] = ttransform<real>(IDENTITY);
    T_global_joint_map["none"] = ttransform<real>(IDENTITY);
    idx_map["none"] = -1;

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

        std::string body_type = link_elem->Attribute("type");
        CollisionShape col_shape;
        std::string obj_filename = "";
        if (body_type == "box") {
            glm::rvec3 size = string_to_vector3d(link_elem->Attribute("size"));
            col_shape = CollisionShape::make_box(size);
        }
        else if (body_type == "sphere") {
            double radius = std::stod(link_elem->Attribute("radius"));
            col_shape = CollisionShape::make_sphere(radius);
        }
        else if (body_type == "mesh") {
            fs::path filepath = fs::path(current_dir) / link_elem->Attribute("obj");
            obj_filename = filepath.string();
            col_shape = CollisionShape::make_mesh_bvh();
        }
        else if (body_type == "mesh_sdf") {
            auto cell_size_str = link_elem->Attribute("cell_size");
            real cell_size;
            if (cell_size_str) {
                cell_size = std::stod(cell_size_str);
            }
            else {
                // Default to 0.5cm grid size
                cell_size = 0.005;
            }
            fs::path filepath = fs::path(current_dir) / link_elem->Attribute("obj");
            obj_filename = filepath.string();
            col_shape = CollisionShape::make_mesh_sdf(cell_size);
        }
        else if (body_type == "capsule") {
            double radius = std::stod(link_elem->Attribute("radius"));
            double height = std::stod(link_elem->Attribute("height"));
            printf("Capsule not supported!");
            return false;
        }

        real density;
        if (link_elem->Attribute("density")) {
            density = std::stod(link_elem->Attribute("density"));
            real volume = col_shape.mass(real(1));
        }
        else if (link_elem->Attribute("mass")) {
            real mass = std::stod(link_elem->Attribute("mass"));
            real volume = col_shape.mass(real(1));
            density = mass / volume;
        }

        ttransform<real> T_global_body;
        T_global_body.R = glmx::exp_mat(string_to_vector3d(link_elem->Attribute("rot")));
        T_global_body.v = string_to_vector3d(link_elem->Attribute("pos"));

        XMLElement* joint_elem = node->FirstChildElement("joint");
        std::string joint_type = joint_elem->Attribute("type");

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
            local_joint_pose = T_global_joint;
        }
        ttransform<real> local_link_pose = T_global_body / T_global_joint;

        link = Link::create(col_shape, density, local_joint_pose, local_link_pose, idx_map[parent_name], {}, obj_filename);

        real kp = 0.0;
        real kd = 0.0;
        auto kp_str = joint_elem->Attribute("kp");
        if (kp_str) {
            kp = std::stod(kp_str);
        }
        auto kd_str = joint_elem->Attribute("kd");
        if (kd_str) {
            kd = std::stod(kd_str);
        }
        if(joint_type == "free" || joint_type == "floating")
        {
            // TODO: Should we also put kd on floating joints?
            joint = Joint::floating();
        }
        else if(joint_type == "ball" || joint_type == "spherical")
        {
            joint = Joint::spherical(kp, kd);
        }
        else if(joint_type == "revolute")
        {
            glm::rvec3 axis = string_to_vector3d(joint_elem->Attribute("axis"));
            if (glm::epsilonEqual<real>(axis.x, 1.0, 1e-8)) {
                joint = Joint::revolute_x(kp, kd);
            }
            else if (glm::epsilonEqual<real>(axis.y, 1.0, 1e-8)) {
                joint = Joint::revolute_y(kp, kd);
            }
            else if (glm::epsilonEqual<real>(axis.z, 1.0, 1e-8)) {
                joint = Joint::revolute_z(kp, kd);
            }
            else {
                std::cout << "Only revolute joints with X, Y, or Z axis supported!" << std::endl;
                return false;
            }
        }

        spec.add_link_and_joint(link, joint, name);
        idx_map[name] = current_idx;
        current_idx++;
    }

    spec.build();
    return true;
}

tinyxml2::XMLError load_from_xml(const char* filename, OUT ArticulatedBodySpec& spec) {
    XMLDocument doc;
    XMLError err = doc.LoadFile(filename);
    if (err != XMLError::XML_SUCCESS) {
        std::cout << "Can't open file : " << filename << std::endl;
        return err;
    }
    // TODO: better error checking
    auto parent_folder = fs::path(filename).parent_path().string();
    if (load_from_xml(doc.RootElement(), parent_folder.c_str(), spec)) {
        return err;
    }
    else {
        return err;
    }
}

tinyxml2::XMLElement* save_to_xml(tinyxml2::XMLDocument& doc, ArticulatedBodySpec& spec) {
    auto el_art = doc.NewElement("articulation");
    el_art->SetAttribute("name", "default");
    el_art->SetAttribute("xform_mode", "global");
    int num_nodes = spec.get_num_joints();
    for (int i = 0; i < num_nodes; i++) {
        auto& link = spec.links[i];
        auto& joint = spec.joints[i];
        auto& name = spec.names[i];
        auto el_node = doc.NewElement("node");
        el_art->InsertEndChild(el_node);
        el_node->SetAttribute("name", name.c_str());
        if (spec.parents[i] == -1) {
            el_node->SetAttribute("parent", "none");
        }
        else {
            el_node->SetAttribute("parent", spec.names[spec.parents[i]].c_str());
        }

        ArticulatedBody art;
        art.init(spec);
        art.forward_kinematics();

        auto el_link = doc.NewElement("link");
        el_node->InsertEndChild(el_link);

        switch (link.col_shape.type) {
            case CollisionShape::Type::Ground:
                el_link->SetAttribute("type", "ground");
                break;
            case CollisionShape::Type::Sphere:
                el_link->SetAttribute("type", "sphere");
                el_link->SetAttribute("radius", link.col_shape.scale[0]);
                break;
            case CollisionShape::Type::Box: {
                el_link->SetAttribute("type", "box");
                auto size_str = to_string(link.col_shape.scale);
                el_link->SetAttribute("size", size_str.c_str());
            } break;
            case CollisionShape::Type::Mesh: {
                el_link->SetAttribute("type", "mesh");
                auto scale_str = to_string(link.col_shape.scale);
                el_link->SetAttribute("scale", scale_str.c_str()); // TODO
            } break;
        }

        real volume = link.col_shape.mass(real(1));
        real density = link.mass / volume;

        el_link->SetAttribute("mass", link.mass);
        auto link_pos_str = to_string(art.get_global_link_trans(i).v);
        el_link->SetAttribute("pos", link_pos_str.c_str());
        auto link_rot_str = to_string(glmx::log_mat(art.get_global_link_trans(i).R));
        el_link->SetAttribute("rot", link_rot_str.c_str());

        auto el_joint = doc.NewElement("joint");
        el_node->InsertEndChild(el_joint);

        switch (joint.type) {
            case JOINT_TYPE_FLOATING: {
                el_joint->SetAttribute("type", "free");
            } break;
            case JOINT_TYPE_PRISMATIC_X: {
                el_joint->SetAttribute("type", "prismatic");
                el_joint->SetAttribute("axis", "1 0 0");
            } break;
            case JOINT_TYPE_PRISMATIC_Y: {
                el_joint->SetAttribute("type", "prismatic");
                el_joint->SetAttribute("axis", "0 1 0");
            } break;
            case JOINT_TYPE_PRISMATIC_Z: {
                el_joint->SetAttribute("type", "prismatic");
                el_joint->SetAttribute("axis", "0 0 1");
            } break;
            case JOINT_TYPE_REVOLUTE_X:  {
                el_joint->SetAttribute("type", "revolute");
                el_joint->SetAttribute("axis", "1 0 0");
            } break;
            case JOINT_TYPE_REVOLUTE_Y: {
                el_joint->SetAttribute("type", "revolute");
                el_joint->SetAttribute("axis", "0 1 0");
            } break;
            case JOINT_TYPE_REVOLUTE_Z: {
                el_joint->SetAttribute("type", "revolute");
                el_joint->SetAttribute("axis", "0 0 1");
            } break;
            case JOINT_TYPE_SPHERICAL: {
                el_joint->SetAttribute("type", "spherical");
            } break;
        }
        el_joint->SetAttribute("kp", joint.kp);
        el_joint->SetAttribute("kd", joint.kd);
        auto joint_pos_str = to_string(art.get_global_joint_trans(i).v);
        el_joint->SetAttribute("pos", joint_pos_str.c_str());
        auto joint_rot_str = to_string(glmx::log_mat(art.get_global_joint_trans(i).R));
        el_joint->SetAttribute("rot", joint_rot_str.c_str());
    }
    return el_art;
}

XMLError save_to_xml(const char* filename, ArticulatedBodySpec& art) {
    XMLDocument doc;
    XMLElement* el_art = save_to_xml(doc, art);
    doc.InsertEndChild(el_art);
    return doc.SaveFile(filename);
}

}
