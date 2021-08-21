//
// Created by lasagnaphil on 8/21/21.
//

#ifndef ARTSIM_WORLD_PBR_RENDER_H
#define ARTSIM_WORLD_PBR_RENDER_H

#include <gengine/PBRenderer.h>

class WorldPBRRender {
private:
    artsim::World* world;

public:
    WorldPBRRender() = default;
    WorldPBRRender(artsim::World* world) : world(world) {}

    void render(PBRenderer& renderer) {
    }
};

#endif //ARTSIM_WORLD_PBR_RENDER_H
