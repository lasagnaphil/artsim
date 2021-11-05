//
// Created by lasagnaphil on 1/23/21.
//

#include "artsim/utils/urdf.h"
#include <pugixml.hpp>
#include <glm/gtx/euler_angles.hpp>

using namespace glmx;

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

void artsim::export_to_urdf(const artsim::ArticulatedBodySpec& art, const char* robot_name, const char* filename) {

    pugi::xml_document doc;
    auto robot_elem = doc.append_child("robot");
    robot_elem.append_attribute("name") = robot_name;

    if (art.floating) {
        auto base_link_elem = robot_elem.append_child("link");
        base_link_elem.append_attribute("name") = "base";
    }

    for (int i = 0; i < art.links.size(); i++) {
        auto& link = art.links[i];
        auto& joint = art.joints[i];
        auto& name = art.names[i];

        ttransform<real> link_origin = link.local_link_pose;
        ttransform<real> joint_origin = link.local_joint_pose;

        glm::tvec3<real> link_origin_xyz = link_origin.v;
        glm::tvec3<real> link_origin_rpy;
        glm::extractEulerAngleXYZ(glm::tmat4x4<real>(link_origin.R),
                link_origin_rpy.x, link_origin_rpy.y, link_origin_rpy.z);
        glm::tvec3<real> joint_origin_xyz = joint_origin.v;
        glm::tvec3<real> joint_origin_rpy;
        glm::extractEulerAngleXYZ(glm::tmat4x4<real>(joint_origin.R),
                joint_origin_rpy.x, joint_origin_rpy.y, joint_origin_rpy.z);

        auto link_elem = robot_elem.append_child("link");
        link_elem.append_attribute("name") = name.c_str();
        {
            auto inertial_elem = link_elem.append_child("inertial");
            {
                auto origin_elem = inertial_elem.append_child("origin");
                origin_elem.append_attribute("xyz") = to_string(link_origin_xyz).c_str();
                origin_elem.append_attribute("rpy") = to_string(link_origin_rpy).c_str();

                auto mass_elem = inertial_elem.append_child("mass");
                mass_elem.append_attribute("value") = to_string(link.mass).c_str();

                auto inertia_elem = inertial_elem.append_child("inertia");
                inertia_elem.append_attribute("ixx") = to_string(link.inertia.xx).c_str();
                inertia_elem.append_attribute("iyy") = to_string(link.inertia.yy).c_str();
                inertia_elem.append_attribute("izz") = to_string(link.inertia.zz).c_str();
                inertia_elem.append_attribute("iyz") = to_string(link.inertia.yz).c_str();
                inertia_elem.append_attribute("izx") = to_string(link.inertia.zx).c_str();
                inertia_elem.append_attribute("ixy") = to_string(link.inertia.xy).c_str();
            }

            auto geometry_elem = link_elem.append_child("geometry");
            switch (link.col_shape.type) {
                case CollisionShape::Type::Box: {
                    auto box_elem = geometry_elem.append_child("box");
                    box_elem.append_attribute("size") = to_string(link.col_shape.scale).c_str();
                } break;
                case CollisionShape::Type::Sphere: {
                    auto sphere_elem = geometry_elem.append_child("sphere");
                    sphere_elem.append_attribute("radius") = to_string(link.col_shape.scale.x).c_str();
                }
            }

            auto visual_elem = link_elem.append_child("visual");
            {
                auto origin_elem = visual_elem.append_child("origin");
                origin_elem.append_attribute("xyz") = to_string(link_origin_xyz).c_str();
                origin_elem.append_attribute("rpy") = to_string(link_origin_rpy).c_str();

                visual_elem.append_copy(geometry_elem);

                // TODO: Add material
            }

            auto collision_elem = link_elem.append_child("collision");
            {
                auto origin_elem = collision_elem.append_child("origin");
                origin_elem.append_attribute("xyz") = to_string(link_origin_xyz).c_str();
                origin_elem.append_attribute("rpy") = to_string(link_origin_rpy).c_str();

                collision_elem.append_copy(geometry_elem);
            }
        }

        auto joint_elem = robot_elem.append_child("joint");
        joint_elem.append_attribute("name") = name.c_str();
        switch (joint.type) {
            case JOINT_TYPE_REVOLUTE_X: case JOINT_TYPE_REVOLUTE_Y: case JOINT_TYPE_REVOLUTE_Z:
                joint_elem.append_attribute("type") = "revolute"; break;
            case JOINT_TYPE_PRISMATIC_X: case JOINT_TYPE_PRISMATIC_Y: case JOINT_TYPE_PRISMATIC_Z:
                joint_elem.append_attribute("type") = "prismatic"; break;
            case JOINT_TYPE_SPHERICAL:
                joint_elem.append_attribute("type") = "spherical"; break;
            case JOINT_TYPE_FLOATING:
                joint_elem.append_attribute("type") = "floating"; break;
        }

        {
            auto origin_elem = joint_elem.append_child("origin");
            origin_elem.append_attribute("xyz") = to_string(joint_origin_xyz).c_str();
            origin_elem.append_attribute("rpy") = to_string(joint_origin_rpy).c_str();

            auto parent_elem = joint_elem.append_child("parent");
            if (link.parent_idx == -1) {
                parent_elem.append_attribute("link") = "base";
            }
            else {
                parent_elem.append_attribute("link") = art.names[link.parent_idx].c_str();
            }

            auto child_elem = joint_elem.append_child("child");
            child_elem.append_attribute("link") = name.c_str();
        }
    }

    doc.save_file(filename);
}
