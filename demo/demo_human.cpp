//
// Created by lasagnaphil on 1/23/21.
//

#include <iostream>
#include <vector>
#include <tinyxml2.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <imgui.h>
#include <implot.h>

#include <artsim/artsim.h>
#include <artsim/articulation_state.h>
#include <artsim/utils/urdf.h>

#include "articulation_render.h"

#include <gengine/App.h>
#include <gengine/InputManager.h>

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

artsim::ArticulatedBody load_human(const char* filename, std::vector<uint32_t>& contact_indices) {
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
            if (glm::epsilonEqual(axis.x, 1.0, 1e-8)) {
                joint = Joint::revolute_x(kp, kd);
            }
            else if (glm::epsilonEqual(axis.y, 1.0, 1e-8)) {
                joint = Joint::revolute_y(kp, kd);
            }
            else if (glm::epsilonEqual(axis.z, 1.0, 1e-8)) {
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

class MyApp : public App {
public:
    MyApp() : App(AppSettings::defaultPBR()) {}

    enum class DemoType {
        Pendulum, Contacts
    };

    void loadResources() {

        FlyCamera* camera = dynamic_cast<FlyCamera*>(this->camera.get());
        Ref<Transform> cameraTransform = camera->transform;
        cameraTransform->move({0.0f, 2.0f, 0.0f});

        pbRenderer.dirLightProjVolume = {
                {-10.f, -10.f, 0.f}, {10.f, 10.f, 100.f}
        };
        pbRenderer.shadowFramebufferSize = {2048, 2048};

        pbRenderer.dirLight.enabled = true;
        pbRenderer.dirLight.direction = glm::normalize(glm::vec3 {2.0f, -3.0f, -2.0f});
        pbRenderer.dirLight.color = glm::vec3(1.0f);

        ground_mat = PBRMaterial::quick(colors::White);
        ground_mesh = Mesh::makePlane(10.0f, 10.0f);

        reset();

        link_mat = PBRMaterial::quick(0.5f * colors::Red);
        joint_mat = PBRMaterial::quick(colors::Green);
        art_render = ArticulationStateRender(&state, link_mat, joint_mat);
    }

    void processInput(SDL_Event &event) override {
    }

    void update(float dt) override {
        auto inputMgr = InputManager::get();

        if (inputMgr->isKeyEntered(SDL_SCANCODE_R)) {
            reset();
        }
        if (inputMgr->isKeyEntered(SDL_SCANCODE_SPACE)) {
            run_simulation = !run_simulation;
        }

        if (run_simulation) {
            auto t1 = std::chrono::high_resolution_clock::now();
            state.simulate(sim_dt, 10);
            auto t2 = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
            float sim_time = 0.001f * duration.count() / 10;
            if (prev_sim_times.size() == 60) {
                prev_sim_times.pop_front();
            }
            prev_sim_times.push_back(sim_time);
        }

        float avg_sim_time = 0.0f;
        for (auto& t : prev_sim_times) {
            avg_sim_time += t;
        }
        avg_sim_time /= prev_sim_times.size();
    }

    void render() override {
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        pbRenderer.queueRender({ground_mesh, ground_mat, rootTransform->getWorldTransform()});
        art_render.render(pbRenderer, imRenderer);

        imRenderer.drawXZSquareGrid(-5.0f, 5.0f, 0.01f, 1.0f, colors::LightGray, true);

        pbRenderer.render();
        imRenderer.render();
    }

    void release() override {
    }

    void reset() {
        art = load_human("demo/resources/human.xml", contact_indices);
        export_to_urdf(art, "human", "demo/resources/human.urdf");

        state = ArticulationState(&art, &material_db, ContactSolverType::PGS, 16);
        state.enable_collision_with_ground = true;
        state.ground_col_enabled_links = contact_indices;
        state.set_root_transform(ttransform<real>(tvec3<real>(0.0f, 1.3f, 0.0f)));
        state.update_transforms();

        art_render = ArticulationStateRender(&state, link_mat, joint_mat);
    }

private:
    ArticulatedBody art;
    MaterialDB material_db;
    ArticulationState state;
    float sim_dt = 1.0f / 600.0f;
    bool run_simulation = true;

    std::vector<uint32_t> contact_indices;

    std::deque<float> prev_sim_times;

    Ref<PBRMaterial> ground_mat;
    Ref<Mesh> ground_mesh;

    Ref<PBRMaterial> link_mat, joint_mat;
    ArticulationStateRender art_render;
};

int main(int argc, char** argv) {
    MyApp app;
    app.load();
    app.startMainLoop();
    app.release();
}
