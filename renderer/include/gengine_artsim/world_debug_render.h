//
// Created by lasagnaphil on 8/17/21.
//

#ifndef ARTSIM_WORLD_DEBUG_RENDER_H
#define ARTSIM_WORLD_DEBUG_RENDER_H

#include <artsim/world.h>

class WorldDebugRender {
private:
    artsim::World* world;

public:
    WorldDebugRender() = default;
    WorldDebugRender(artsim::World* world) : world(world) {}

    void render(DebugRenderer& debug_renderer) {
        auto& contact_points = world->get_contact_points();
        for (auto& cp : contact_points) {
            glm::mat3 R = glm::mat3(cp.tangent1, cp.tangent2, cp.normal);
            glm::mat4 transform = glm::translate(cp.pos) * glm::mat4(R);
            debug_renderer.drawAxisTriad(transform, 0.01f, 0.1f, true);
            glm::rvec3 disp = R * cp.lam;
            debug_renderer.drawArrow(cp.pos, cp.pos + 10.0f * disp, colors::Green, 1.0f * glm::length(disp), true);
            // debug_renderer.drawArrow(cp.pos, cp.pos - 10.0f * disp, colors::Green, 1.0f * glm::length(disp), true);
        }
    }
};

#endif //ARTSIM_WORLD_DEBUG_RENDER_H
