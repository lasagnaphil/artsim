//
// Created by lasagnaphil on 19. 3. 14.
//

#include "gengine/Texture.h"

#include <iostream>
#include <cmath>

Ref<Texture> Texture::fromImage(Ref<Image> image) {
    Ref<Texture> tex = Resources::make<Texture>();
    tex->loadFromImage(image);
    return tex;
}

Ref<Texture> Texture::fromSubImage(Ref<Image> image, int xoffset, int yoffset, int width, int height) {
    Ref<Texture> tex = Resources::make<Texture>();
    tex->loadFromSubImage(image, xoffset, yoffset, width, height);
    return tex;
}

Ref<Texture> Texture::fromNew(uint32_t width, uint32_t height,
                              GLuint imageFormat, GLuint internalFormat,
                              TextureGLParams gl_params) {
    Ref<Texture> tex = Resources::make<Texture>();
    tex->width = width;
    tex->height = height;
    tex->wrapS = gl_params.wrapS;
    tex->wrapT = gl_params.wrapT;
    tex->filterMin = gl_params.filterMin;
    tex->filterMax = gl_params.filterMax;
    tex->imageFormat = imageFormat;
    tex->internalFormat = internalFormat;

    glGenTextures(1, &tex->id);
    glBindTexture(GL_TEXTURE_2D, tex->id);
    glTexImage2D(GL_TEXTURE_2D,
                 0, tex->internalFormat,
                 width, height,
                 0, tex->imageFormat, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, gl_params.filterMin);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, gl_params.filterMax);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, gl_params.wrapS);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, gl_params.wrapT);
    glBindTexture(GL_TEXTURE_2D, 0);

    return tex;
}

Ref<Texture> Texture::fromSingleColor(glm::vec3 color) {
    Ref<Texture> tex = Resources::make<Texture>();
    tex->width = 1;
    tex->height = 1;
    tex->wrapS = GL_REPEAT;
    tex->wrapT = GL_REPEAT;
    tex->filterMin = GL_LINEAR_MIPMAP_LINEAR;
    tex->filterMax = GL_LINEAR;
    tex->imageFormat = GL_RGB;
    tex->internalFormat = GL_RGB;

    char data[3];
    data[0] = std::max(0, std::min(255, (int)floorf(color.r * 256.0f)));
    data[1] = std::max(0, std::min(255, (int)floorf(color.g * 256.0f)));
    data[2] = std::max(0, std::min(255, (int)floorf(color.b * 256.0f)));

    glGenTextures(1, &tex->id);
    glBindTexture(GL_TEXTURE_2D, tex->id);
    glTexImage2D(GL_TEXTURE_2D,
                 0, tex->internalFormat,
                 tex->width, tex->height,
                 0, tex->imageFormat, GL_UNSIGNED_BYTE, data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    return tex;
}

void Texture::loadFromImage(Ref<Image> image) {
    width = image->width;
    height = image->height;
    wrapS = GL_REPEAT;
    wrapT = GL_REPEAT;
    filterMin = GL_LINEAR_MIPMAP_LINEAR;
    filterMax = GL_LINEAR;

    int nrComponents = image->nrChannels;
    if (nrComponents == 1) {
        imageFormat = GL_RED;
        internalFormat = GL_RED;
    }
    else if (nrComponents == 3) {
        imageFormat = GL_RGB;
        internalFormat = GL_RGB;
    }
    else if (nrComponents == 4) {
        imageFormat = GL_RGBA;
        internalFormat = GL_RGBA;
    }

    glGenTextures(1, &id);
    glBindTexture(GL_TEXTURE_2D, id);

    glTexImage2D(GL_TEXTURE_2D,
                 0, internalFormat,
                 width, height,
                 0, imageFormat, GL_UNSIGNED_BYTE, image->data);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapS);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filterMin);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filterMax);

    glGenerateMipmap(GL_TEXTURE_2D);

    glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::loadFromSubImage(Ref<Image> image, int xoffset, int yoffset, int width, int height) {
    width = image->width;
    height = image->height;

    wrapS = GL_REPEAT;
    wrapT = GL_REPEAT;
    filterMin = GL_LINEAR_MIPMAP_LINEAR;
    filterMax = GL_LINEAR;

    int nrComponents = image->nrChannels;
    if (nrComponents == 1) {
        imageFormat = GL_RED;
        internalFormat = GL_RED;
    }
    else if (nrComponents == 3) {
        imageFormat = GL_RGB;
        internalFormat = GL_RGB;
    }
    else if (nrComponents == 4) {
        imageFormat = GL_RGBA;
        internalFormat = GL_RGBA;
    }

    glGenTextures(1, &id);
    glBindTexture(GL_TEXTURE_2D, id);

    glTexImage2D(GL_TEXTURE_2D,
                 0, internalFormat,
                 width, height,
                 0, imageFormat, GL_UNSIGNED_BYTE, image->data);

    glTexSubImage2D(GL_TEXTURE_2D, 0,
                    xoffset, yoffset, width, height,
                    imageFormat, GL_UNSIGNED_BYTE, image->data);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapS);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filterMin);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filterMax);

    glGenerateMipmap(GL_TEXTURE_2D);

    glBindTexture(GL_TEXTURE_2D, 0);
}

Ref<Image> Texture::saveToImage(int numChannels, GLenum format, GLenum type) {
    auto image = Image::fromEmpty(width, height, numChannels);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, id);
    glGetTexImage(GL_TEXTURE_2D, 0, format, type, image->data);
    return image;
}

void Texture::bind() {
    glBindTexture(GL_TEXTURE_2D, id);
}

void Texture::dispose() {
    if (id != 0) {
        glDeleteTextures(1, &id);
    }
}
