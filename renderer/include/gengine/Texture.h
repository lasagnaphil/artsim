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

struct TextureGLParams {
    GLint wrapS = GL_REPEAT;
    GLint wrapT = GL_REPEAT;
    GLint filterMin = GL_LINEAR_MIPMAP_LINEAR;
    GLint filterMax = GL_LINEAR;
};

struct Texture {
    GLuint id = 0;
    GLuint width, height;
    GLuint internalFormat;
    GLuint imageFormat;
    GLint wrapS;
    GLint wrapT;
    GLint filterMin;
    GLint filterMax;

    Texture() = default;
    ~Texture() { dispose(); }
    static Ref<Texture> fromImage(Ref<Image> image);
    static Ref<Texture> fromSubImage(Ref<Image> image, int xoffset, int yoffset, int width, int height);
    static Ref<Texture> fromNew(uint32_t width, uint32_t height,
                                GLuint imageFormat = GL_RGB, GLuint internalFormat = GL_RGB,
                                TextureGLParams gl_params = TextureGLParams());
    static Ref<Texture> fromSingleColor(glm::vec3 color);
    void loadFromImage(Ref<Image> image);
    void loadFromSubImage(Ref<Image> image, int xoffset, int yoffset, int width, int height);
    Ref<Image> saveToImage(int numChannels = 4, GLenum format = GL_RGBA, GLenum type = GL_UNSIGNED_BYTE);
    void dispose();

    void bind();
};

#endif //MOTION_EDITING_TEXTURE_H
