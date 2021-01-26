//
// Created by lasagnaphil on 1/26/21.
//

#ifndef ARTSIM_RAYLIB_BULLET_RENDERER_H
#define ARTSIM_RAYLIB_BULLET_RENDERER_H

#include "LinearMath/btIDebugDraw.h"
#include <raylib.h>

Vector3 rayconv(const btVector3& v) {
    return (Vector3){(float)v.x(), (float)v.y(), (float)v.z()};
}

Color rayconv_color(const btVector3& v) {
    return ColorFromNormalized((Vector4){(float)v.x(), (float)v.y(), (float)v.z(), 1});
}

class RaylibBulletRenderer : public btIDebugDraw {
public:
    int debugMode = 1;
    void drawLine(const btVector3& from, const btVector3& to, const btVector3& color) override {
        DrawLine3D(rayconv(from), rayconv(to), rayconv_color(color));
    }

    void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime,
                          const btVector3& color) override {
        DrawSphere(rayconv(PointOnB), 0.02f, rayconv_color(color));
        DrawLine3D(rayconv(PointOnB), rayconv(PointOnB + 0.2f * normalOnB), rayconv_color(color));
    }

    void reportErrorWarning(const char* warningString) override {
        printf("%s\n", warningString);
    }

    void draw3dText(const btVector3& location, const char* textString) override {
    }

    void setDebugMode(int debugMode) override {
        this->debugMode = debugMode;
    }

    int getDebugMode() const override {
        return debugMode;
    }
};

#endif //ARTSIM_RAYLIB_BULLET_RENDERER_H
