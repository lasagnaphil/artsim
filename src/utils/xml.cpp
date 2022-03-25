//
// Created by lasagnaphil on 21. 2. 17..
//

#include "artsim/utils/xml.h"

#include <iostream>
#include <sstream>
#include <pugixml.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <filesystem>

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

bool load_from_xml_legacy(pugi::xml_node root_el, OUT ArticulatedBodySpec& art) {
    std::unordered_map<std::string, ttransform<real>> T_global_body_map;
    std::unordered_map<std::string, ttransform<real>> T_global_joint_map;
    std::unordered_map<std::string, int> idx_map;

    T_global_body_map["None"] = ttransform<real>(IDENTITY);
    T_global_joint_map["None"] = ttransform<real>(IDENTITY);
    idx_map["None"] = -1;

    auto skeleton_elem = root_el;
    std::string skel_name = skeleton_elem.attribute("name").as_string();

    int current_idx = 0;
    for (auto node : skeleton_elem.child("Node")) {
        artsim::Joint joint;
        artsim::Link link;

        std::string name = node.attribute("name").as_string();

        std::string parent_name = node.attribute("parent").as_string();

        auto body_elem = node.child("Body");
        std::string obj_file = "None";
        if(body_elem.attribute("obj"))
            obj_file = body_elem.attribute("obj").as_string();

        real mass = body_elem.attribute("mass").as_double();

        std::string body_type = body_elem.attribute("type").as_string();
        CollisionShape shape;
        if (body_type == "Box") {
            glm::rvec3 size = string_to_vector3d(body_elem.attribute("size").as_string());
            shape = CollisionShape::make_box(size);
        }
        else if (body_type == "Sphere") {
            double radius = body_elem.attribute("radius").as_double();
            shape = CollisionShape::make_sphere(radius);
        }
        else if (body_type == "Capsule") {
            double radius = body_elem.attribute("radius").as_double();
            double height = body_elem.attribute("height").as_double();
            printf("Capsule not supported!\n");
            exit(EXIT_FAILURE);
        }

        real volume = shape.mass(1);
        real density = mass / volume;
        auto inertia = shape.inertia(density);

        bool contact = false;
        if(body_elem.attribute("contact") != nullptr){
            std::string c = body_elem.attribute("contact").as_string();
            if(c == "On") contact = true;
        }

        ttransform<real> T_global_body;
        T_global_body.R = string_to_matrix3d(body_elem.child("Transformation").attribute("linear").as_string());
        T_global_body.v = string_to_vector3d(body_elem.child("Transformation").attribute("translation").as_string());

        auto joint_elem = node.child("Joint");
        std::string joint_type = joint_elem.attribute("type").as_string();
        ttransform<real> T_global_joint;
        T_global_joint.R = string_to_matrix3d(joint_elem.child("Transformation").attribute("linear").as_string());
        T_global_joint.v = string_to_vector3d(joint_elem.child("Transformation").attribute("translation").as_string());

        T_global_body_map[name] = T_global_body;
        T_global_joint_map[name] = T_global_joint;

        ttransform<real> local_joint_pose = T_global_joint / T_global_joint_map[parent_name];
        ttransform<real> local_link_pose = T_global_body / T_global_joint;

        link = Link::create(shape, mass, inertia, local_joint_pose, local_link_pose, idx_map[parent_name], {});

        real kp = joint_elem.attribute("kp").as_double(0.0);
        real kd = joint_elem.attribute("kd").as_double(0.0);
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
            glm::rvec3 axis = string_to_vector3d(joint_elem.attribute("axis").as_string());
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

pugi::xml_parse_result load_from_xml_legacy(const char* filename, OUT ArticulatedBodySpec& art) {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filename);
    if (!result) {
        std::cout << "Can't open file : " << filename << std::endl;
        return result;
    }
    // TODO: better error checking
    if (load_from_xml_legacy(doc.document_element(), art)) {
        return result;
    }
    else {
        return result;
    }
}

bool load_from_xml(pugi::xml_node art_elem, const char* current_dir, OUT ArticulatedBodySpec& spec) {
    std::unordered_map<std::string, ttransform<real>> T_global_body_map;
    std::unordered_map<std::string, ttransform<real>> T_global_joint_map;
    std::unordered_map<std::string, int> idx_map;

    T_global_body_map["none"] = ttransform<real>(IDENTITY);
    T_global_joint_map["none"] = ttransform<real>(IDENTITY);
    idx_map["none"] = -1;

    std::string art_name = art_elem.attribute("name").as_string();
    std::string art_xform_mode = art_elem.attribute("xform_mode").as_string();
    if (art_xform_mode != "global") {
        std::cout << "Only xform_mode = global supported!" << std::endl;
        exit(EXIT_FAILURE);
    }

    int current_idx = 0;
    for (auto node : art_elem.children("node"))
    {
        artsim::Joint joint;
        artsim::Link link;

        std::string name = node.attribute("name").as_string();

        std::string parent_name = node.attribute("parent").as_string();

        auto link_elem = node.child("link");

        std::string body_type = link_elem.attribute("type").as_string();
        CollisionShape col_shape;
        std::string obj_filename = "";
        if (body_type == "box") {
            glm::rvec3 size = string_to_vector3d(link_elem.attribute("size").as_string());
            col_shape = CollisionShape::make_box(size);
        }
        else if (body_type == "sphere") {
            double radius = link_elem.attribute("radius").as_double();
            col_shape = CollisionShape::make_sphere(radius);
        }
        else if (body_type == "mesh") {
            fs::path filepath = fs::path(current_dir) / link_elem.attribute("obj").as_string();
            obj_filename = filepath.string();
            col_shape = CollisionShape::make_mesh_bvh();
        }
        else if (body_type == "mesh_sdf") {
            auto cell_size_str = link_elem.attribute("cell_size");
            real cell_size;
            if (cell_size_str) {
                cell_size = cell_size_str.as_double();
            }
            else {
                // Default to 0.5cm grid size
                cell_size = 0.005;
            }
            fs::path filepath = fs::path(current_dir) / link_elem.attribute("obj").as_string();
            obj_filename = filepath.string();
            col_shape = CollisionShape::make_mesh_sdf(cell_size);
        }
        else if (body_type == "capsule") {
            double radius = link_elem.attribute("radius").as_double();
            double height = link_elem.attribute("height").as_double();
            printf("Capsule not supported!");
            return false;
        }

        real density;
        if (link_elem.attribute("density")) {
            density = link_elem.attribute("density").as_double();
        }
        else if (link_elem.attribute("mass")) {
            real mass = link_elem.attribute("mass").as_double();
            real volume = col_shape.mass(real(1));
            density = mass / volume;
        }

        ttransform<real> T_global_body;
        T_global_body.R = glmx::exp_mat(string_to_vector3d(link_elem.attribute("rot").as_string()));
        T_global_body.v = string_to_vector3d(link_elem.attribute("pos").as_string());

        auto joint_elem = node.child("joint");
        std::string joint_type = joint_elem.attribute("type").as_string();

        ttransform<real> T_global_joint;
        T_global_joint.R = glmx::exp_mat(string_to_vector3d(joint_elem.attribute("rot").as_string()));
        T_global_joint.v = string_to_vector3d(joint_elem.attribute("pos").as_string());

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

        real kp = joint_elem.attribute("kp").as_double();
        real kd = joint_elem.attribute("kd").as_double();

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
            glm::rvec3 axis = string_to_vector3d(joint_elem.attribute("axis").as_string());
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

    // Load articulation state
    auto initial_state_elem = art_elem.child("initial_state");
    if (initial_state_elem != nullptr) {
        std::stringstream ss(initial_state_elem.text().as_string());
        std::string token;
        while (ss >> token) {
            spec.initial_state.push_back(std::stod(token));
        }
    }

    spec.build();
    return true;
}

pugi::xml_parse_result load_from_xml(const char* filename, OUT ArticulatedBodySpec& spec) {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filename);
    if (!result) {
        std::cout << "Can't open file : " << filename << std::endl;
        return result;
    }
    // TODO: better error checking
    auto parent_folder = fs::path(filename).parent_path().string();
    if (load_from_xml(doc.document_element(), parent_folder.c_str(), spec)) {
        return result;
    }
    else {
        return result;
    }
}

pugi::xml_node save_to_xml(pugi::xml_document& doc, ArticulatedBodySpec& spec) {
    auto el_art = doc.append_child("articulation");
    el_art.append_attribute("name") = "default";
    el_art.append_attribute("xform_mode") = "global";
    int num_nodes = spec.get_num_joints();
    for (int i = 0; i < num_nodes; i++) {
        auto& link = spec.links[i];
        auto& joint = spec.joints[i];
        auto& name = spec.names[i];
        auto el_node = el_art.append_child("node");
        el_node.append_attribute("name") = name.c_str();
        if (spec.parents[i] == -1) {
            el_node.append_attribute("parent") = "none";
        }
        else {
            el_node.append_attribute("parent") = spec.names[spec.parents[i]].c_str();
        }

        ArticulatedBody art;
        art.init(spec);
        art.forward_kinematics();

        auto el_link = el_art.append_child("link");

        switch (link.col_shape.type) {
            case CollisionShape::Type::Ground:
                el_link.append_attribute("type") = "ground";
                break;
            case CollisionShape::Type::Sphere:
                el_link.append_attribute("type") = "sphere";
                el_link.append_attribute("radius") = link.col_shape.scale[0];
                break;
            case CollisionShape::Type::Box: {
                el_link.append_attribute("type") = "box";
                auto size_str = to_string(link.col_shape.scale);
                el_link.append_attribute("size") = size_str.c_str();
            } break;
            case CollisionShape::Type::Mesh: {
                el_link.append_attribute("type") = "mesh";
                auto scale_str = to_string(link.col_shape.scale);
                el_link.append_attribute("scale") = scale_str.c_str(); // TODO
            } break;
        }

        real volume = link.col_shape.mass(real(1));
        real density = link.mass / volume;

        el_link.append_attribute("mass") = link.mass;
        auto link_pos_str = to_string(art.get_global_link_trans(i).v);
        el_link.append_attribute("pos") = link_pos_str.c_str();
        auto link_rot_str = to_string(glmx::log_mat(art.get_global_link_trans(i).R));
        el_link.append_attribute("rot") = link_rot_str.c_str();

        auto el_joint = el_art.append_child("joint");

        switch (joint.type) {
            case JOINT_TYPE_FLOATING: {
                el_joint.append_attribute("type") = "free";
            } break;
            case JOINT_TYPE_PRISMATIC_X: {
                el_joint.append_attribute("type") = "prismatic";
                el_joint.append_attribute("axis") = "1 0 0";
            } break;
            case JOINT_TYPE_PRISMATIC_Y: {
                el_joint.append_attribute("type") = "prismatic";
                el_joint.append_attribute("axis") = "0 1 0";
            } break;
            case JOINT_TYPE_PRISMATIC_Z: {
                el_joint.append_attribute("type") = "prismatic";
                el_joint.append_attribute("axis") = "0 0 1";
            } break;
            case JOINT_TYPE_REVOLUTE_X:  {
                el_joint.append_attribute("type") = "revolute";
                el_joint.append_attribute("axis") = "1 0 0";
            } break;
            case JOINT_TYPE_REVOLUTE_Y: {
                el_joint.append_attribute("type") = "revolute";
                el_joint.append_attribute("axis") = "0 1 0";
            } break;
            case JOINT_TYPE_REVOLUTE_Z: {
                el_joint.append_attribute("type") = "revolute";
                el_joint.append_attribute("axis") = "0 0 1";
            } break;
            case JOINT_TYPE_SPHERICAL: {
                el_joint.append_attribute("type") = "spherical";
            } break;
        }
        el_joint.append_attribute("kp") = joint.kp;
        el_joint.append_attribute("kd") = joint.kd;
        auto joint_pos_str = to_string(art.get_global_joint_trans(i).v);
        el_joint.append_attribute("pos") = joint_pos_str.c_str();
        auto joint_rot_str = to_string(glmx::log_mat(art.get_global_joint_trans(i).R));
        el_joint.append_attribute("rot") = joint_rot_str.c_str();
    }
    return el_art;
}

void save_to_xml(const char* filename, ArticulatedBodySpec& art) {
    pugi::xml_document doc;
    auto el_art = save_to_xml(doc, art);
    doc.save_file(filename);
}

}
