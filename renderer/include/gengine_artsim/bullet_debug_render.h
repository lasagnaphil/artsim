//
// Created by lasagnaphil on 21. 3. 31..
//

#ifndef ARTSIM_BULLET_DEBUG_RENDER_H
#define ARTSIM_BULLET_DEBUG_RENDER_H

#include "LinearMath/btIDebugDraw.h"
#include "gengine/DebugRenderer.h"
#include <artsim/artsim.h>
#include <artsim/math/bullet.h>

class BulletDebugRenderer : public btIDebugDraw {
    DebugRenderer* debugRenderer;

public:
    BulletDebugRenderer(DebugRenderer* debugRenderer = nullptr) : debugRenderer(debugRenderer) {}

    DefaultColors getDefaultColors() const override
    {
        DefaultColors colors;
        colors.m_activeObject = {0, 0, 0};
        return colors;
    }
    void drawLine(const btVector3& from, const btVector3& to, const btVector3& color) override {
        debugRenderer->drawLine(artsim::glmconv(from), artsim::glmconv(to), artsim::glmconv(color), false);
    }

    void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance,
                          int lifeTime, const btVector3& color) override {
        debugRenderer->drawSphere(artsim::glmconv(PointOnB), artsim::glmconv(color), 0.1f, false);
        debugRenderer->drawArrow(artsim::glmconv(PointOnB), artsim::glmconv(PointOnB + distance*normalOnB),
                                 artsim::glmconv(color), 0.01f, false);
    }

    void reportErrorWarning(const char* warningString) override {
        puts(warningString);
        puts("\n");
    }

    void draw3dText(const btVector3& location, const char* textString) override {
    }

    void setDebugMode(int debugMode) override {}

    int getDebugMode() const override { return true; }

};

#endif //ARTSIM_BULLET_DEBUG_RENDER_H
