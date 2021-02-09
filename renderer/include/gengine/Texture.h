//
// Created by lasagnaphil on 19. 3. 14.
//

#ifndef MOTION_EDITING_TEXTURE_H
#define MOTION_EDITING_TEXTURE_H

#include "gengine/Image.h"
#include "gengine/Arena.h"
#include "gengine/IDisposable.h"

#include <glm/vec3.hpp>
#include <glad/glad.h>
#include <string>

struct Texture {
    GLuint id = 0;
    GLuint width, height;
    GLuint internalFormat;
    GLuint imageFormat;

    GLuint wrapS;
    GLuint wrapT;
    GLuint filterMin;
    GLuint filterMax;

    Texture() = default;
    static Ref<Texture> fromImage(Ref<Image> image);
    static Ref<Texture> fromNew(uint32_t width, uint32_t height);
    static Ref<Texture> fromSingleColor(glm::vec3 color);
    void loadFromImage(Ref<Image> image);
    void dispose();

    void bind();
};

#endif //MOTION_EDITING_TEXTURE_H
