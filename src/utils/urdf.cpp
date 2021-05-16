//
// Created by lasagnaphil on 1/23/21.
//

#include "artsim/utils/urdf.h"
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
    using namespace tinyxml2;

    XMLDocument doc;
    XMLElement* robot_elem = doc.NewElement("robot");
    robot_elem->SetAttribute("name", robot_name);
    doc.InsertEndChild(robot_elem);

    if (art.floating) {
        XMLElement* base_link_elem = doc.NewElement("link");
        base_link_elem->SetAttribute("name", "base");
        robot_elem->InsertEndChild(base_link_elem);
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

        XMLElement* link_elem = doc.NewElement("link");
        link_elem->SetAttribute("name", name.c_str());
        robot_elem->InsertEndChild(link_elem);
        {
            XMLElement* inertial_elem = doc.NewElement("inertial");
            link_elem->InsertEndChild(inertial_elem);
            {
                XMLElement* origin_elem = doc.NewElement("origin");
                origin_elem->SetAttribute("xyz", to_string(link_origin_xyz).c_str());
                origin_elem->SetAttribute("rpy", to_string(link_origin_rpy).c_str());
                inertial_elem->InsertEndChild(origin_elem);

                XMLElement* mass_elem = doc.NewElement("mass");
                mass_elem->SetAttribute("value", to_string(link.mass).c_str());
                inertial_elem->InsertEndChild(mass_elem);

                XMLElement* inertia_elem = doc.NewElement("inertia");
                inertia_elem->SetAttribute("ixx", to_string(link.inertia.xx).c_str());
                inertia_elem->SetAttribute("iyy", to_string(link.inertia.yy).c_str());
                inertia_elem->SetAttribute("izz", to_string(link.inertia.zz).c_str());
                inertia_elem->SetAttribute("iyz", to_string(link.inertia.yz).c_str());
                inertia_elem->SetAttribute("izx", to_string(link.inertia.zx).c_str());
                inertia_elem->SetAttribute("ixy", to_string(link.inertia.xy).c_str());
                inertial_elem->InsertEndChild(inertia_elem);
            }

            XMLElement* geometry_elem = doc.NewElement("geometry");
            switch (link.col_shape.type) {
                case CollisionShape::Type::Box: {
                    XMLElement* box_elem = doc.NewElement("box");
                    box_elem->SetAttribute("size", to_string(link.col_shape.scale).c_str());
                    geometry_elem->InsertEndChild(box_elem);
                } break;
                case CollisionShape::Type::Sphere: {
                    XMLElement* sphere_elem = doc.NewElement("sphere");
                    sphere_elem->SetAttribute("radius", to_string(link.col_shape.scale.x).c_str());
                    geometry_elem->InsertEndChild(sphere_elem);
                }
            }

            XMLElement* visual_elem = doc.NewElement("visual");
            link_elem->InsertEndChild(visual_elem);
            {
                XMLElement* origin_elem = doc.NewElement("origin");
                origin_elem->SetAttribute("xyz", to_string(link_origin_xyz).c_str());
                origin_elem->SetAttribute("rpy", to_string(link_origin_rpy).c_str());
                visual_elem->InsertEndChild(origin_elem);

                visual_elem->InsertEndChild(geometry_elem->DeepClone(&doc));

                // TODO: Add material
            }

            XMLElement* collision_elem = doc.NewElement("collision");
            link_elem->InsertEndChild(collision_elem);
            {
                XMLElement* origin_elem = doc.NewElement("origin");
                origin_elem->SetAttribute("xyz", to_string(link_origin_xyz).c_str());
                origin_elem->SetAttribute("rpy", to_string(link_origin_rpy).c_str());
                collision_elem->InsertEndChild(origin_elem);

                collision_elem->InsertEndChild(geometry_elem->DeepClone(&doc));
            }
        }

        XMLElement* joint_elem = doc.NewElement("joint");
        joint_elem->SetAttribute("name", name.c_str());
        switch (joint.type) {
            case JOINT_TYPE_REVOLUTE_X: case JOINT_TYPE_REVOLUTE_Y: case JOINT_TYPE_REVOLUTE_Z:
                joint_elem->SetAttribute("type", "revolute"); break;
            case JOINT_TYPE_PRISMATIC_X: case JOINT_TYPE_PRISMATIC_Y: case JOINT_TYPE_PRISMATIC_Z:
                joint_elem->SetAttribute("type", "prismatic"); break;
            case JOINT_TYPE_SPHERICAL:
                joint_elem->SetAttribute("type", "spherical"); break;
            case JOINT_TYPE_FLOATING:
                joint_elem->SetAttribute("type", "floating"); break;
        }
        robot_elem->InsertEndChild(joint_elem);
        {
            XMLElement* origin_elem = doc.NewElement("origin");
            origin_elem->SetAttribute("xyz", to_string(joint_origin_xyz).c_str());
            origin_elem->SetAttribute("rpy", to_string(joint_origin_rpy).c_str());
            joint_elem->InsertEndChild(origin_elem);

            XMLElement* parent_elem = doc.NewElement("parent");
            if (link.parent_idx == -1) {
                parent_elem->SetAttribute("link", "base");
            }
            else {
                parent_elem->SetAttribute("link", art.names[link.parent_idx].c_str());
            }
            joint_elem->InsertEndChild(parent_elem);

            XMLElement* child_elem = doc.NewElement("child");
            child_elem->SetAttribute("link", name.c_str());
            joint_elem->InsertEndChild(child_elem);
        }
    }

    doc.SaveFile(filename);
}
