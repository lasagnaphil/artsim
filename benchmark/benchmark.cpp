//
// Created by lasagnaphil on 8/18/21.
//

#include <artsim/world.h>
#include <artsim/utils/example_articulations.h>
#include <artsim/utils/xml.h>

#include <map>
#include <pugixml.hpp>

using namespace artsim;

int main(int argc, char** argv) {
    std::map<std::string, ArticulatedBodySpec> articulations = {
            {"01. single link pendulum revolute", examples::create_single_pendulum_link(false)},
            {"02. single link pendulum spherical", examples::create_single_pendulum_link(true)},
            {"03. double ball pendulum revolute", examples::create_double_pendulum_ball(false)},
            {"04. double link pendulum revolute", examples::create_double_pendulum_link(false)},
            {"05. double link pendulum spherical", examples::create_double_pendulum_link(true)},
            {"06. triple link pendulum revolute", examples::create_triple_pendulum_link(false)},
            {"07. triple link pendulum spherical", examples::create_triple_pendulum_link(true)},
            {"08. furuta pendulum revolute", examples::create_furuta_pendulum(false)},
            {"09. furuta pendulum spherical", examples::create_furuta_pendulum(true)},
            {"10. 5 link tree revolute", examples::create_5_link_tree(false)},
            {"11. 5 link tree spherical", examples::create_5_link_tree(true)},
            {"12. 13 link tree revolute", examples::create_13_link_tree(false)},
            {"13. 13 link tree spherical", examples::create_13_link_tree(true)},
            {"14. floating single link", examples::create_free_link(1, false)},
            {"15. floating double link revolute", examples::create_free_link(2, false)},
            {"16. floating double link spherical", examples::create_free_link(2, true)},
    };

    printf("Simple benchmarks: \n");
    for (auto& [name, spec] : articulations) {
        ArticulatedBody art;
        art.init(spec);
        art.randomize_positions();
        int num_vel_dofs = art.get_num_vel_dofs();

        real g = 9.81f;
        real dt = 1.0f / 600.0f;
        tvec3<real> gravity = {0, -g, 0};

        // Performance comparison
        const int num_iters = 100000;

        auto t1 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < num_iters; i++) {
            art.forward_dynamics(gravity, dt);
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);

        printf("%s\n", name.c_str());
        printf("%d iters of featherstone forward dynamics: %d microsecs\n", num_iters, duration.count());
    }

    printf("Humanoid contact benchmark: \n");
    {
        real g = 9.81f;
        real dt = 1.0f / 600.0f;
        tvec3<real> gravity = {0, -g, 0};

        World world;
        WorldConfig world_cfg;
        world_cfg.dt = dt;
        world_cfg.gravity = gravity;
        world_cfg.max_vel_iters = 8;
        world_cfg.max_pos_iters = 2;
        world.init(world_cfg);
        auto default_mat_id = world.add_material(1.0f, 0.0f, 0.00f);

        world.add_plane(default_mat_id);

        ArticulatedBodySpec art_spec;
        auto xml_load_result = load_from_xml("demo/resources/human.xml", art_spec);
        if (xml_load_result.status == pugi::xml_parse_status::status_ok) {
            printf("Failed to load articulation!\n");
            exit(EXIT_FAILURE);
        }

        auto art_id = world.add_articulated_body(art_spec, default_mat_id);
        auto art = world.get_articulated_body(art_id);

        art->set_root_transform(glmx::ttransform<real>(tvec3<real>(0.0f, 1.3f, 0.0f)));
        art->forward_kinematics();

        const int num_iters = 10000;

        auto t1 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < num_iters; i++) {
            world.simulate();
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);

        printf("%d iters of simulation: %d microsecs\n", num_iters, duration.count());
    }

    return 0;
}