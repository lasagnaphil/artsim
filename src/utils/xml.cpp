//
// Created by lasagnaphil on 21. 2. 17..
//

#include "artsim/utils/xml.h"

#include <iostream>
#include <tinyxml2.h>
#include <glm/gtc/type_ptr.hpp>

using namespace tinyxml2;
using namespace artsim;
using namespace glmx;

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

ArticulatedBody artsim::load_from_xml(const char* filename, std::vector<uint32_t>& contact_indices) {
    artsim::ArticulatedBody art;

    std::unordered_map<std::string, ttransform<real>> T_global_body_map;
    std::unordered_map<std::string, ttransform<real>> T_global_joint_map;
    std::unordered_map<std::string, int> idx_map;

    T_global_body_map["None"] = ttransform<real>(IDENTITY);
    T_global_joint_map["None"] = ttransform<real>(IDENTITY);
    idx_map["None"] = -1;

    XMLDocument doc;
    if (doc.LoadFile(filename)) {
        std::cout << "Can't open file : " << filename << std::endl;
        exit(EXIT_FAILURE);
    }

    XMLElement *skeleton_elem = doc.FirstChildElement("Skeleton");
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
            glm::tvec3<real> size = string_to_vector3d(body_elem->Attribute("size"));
            shape = CollisionShape::make_box(size);
        }
        else if (body_type == "Sphere") {
            double radius = std::stod(body_elem->Attribute("radius"));
            shape = CollisionShape::make_sphere(radius);
        }
        else if (body_type == "Capsule") {
            double radius = std::stod(body_elem->Attribute("radius"));
            double height = std::stod(body_elem->Attribute("height"));
            printf("Capsule not supported!");
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
        if (contact) {
            contact_indices.push_back(current_idx);
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

        ttransform<real> local_joint_pose;
        if (parent_name != "None") {
            local_joint_pose = T_global_joint / T_global_joint_map[parent_name];
        }
        else {
            local_joint_pose = ttransform<real>(IDENTITY);
        }
        ttransform<real> local_link_pose = T_global_body / T_global_joint;

        link = Link::create(inertia, mass, shape, local_joint_pose, local_link_pose, idx_map[parent_name], {});

        const real kp = 0.0;
        const real kd = 0.4;
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
            glm::tvec3<real> axis = string_to_vector3d(joint_elem->Attribute("axis"));
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

    art.setup();
    return art;
}
