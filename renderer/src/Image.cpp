//
// Created by lasagnaphil on 19. 3. 14.
//

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include <iostream>

#include "gengine/Image.h"
#include "gengine/Arena.h"

Ref<Image> Image::fromFile(const std::string& filename, int desiredChannels){
    Ref<Image> image = Resources::make<Image>();

    image->filename = filename;
    image->data = stbi_load(filename.c_str(), &image->width, &image->height, &image->nrChannels, desiredChannels);
    image->is_ptr_owned = true;
    if (!image->data) {
        std::cerr << "Failed to load image " << filename << "!\n";
        exit(EXIT_FAILURE);
    }
    image->desiredChannels = desiredChannels;

    return image;
}

Ref<Image> Image::fromEmpty(int width, int height, int nrChannels) {
    Ref<Image> image = Resources::make<Image>();
    image->width = width;
    image->height = height;
    image->nrChannels = image->desiredChannels = nrChannels;
    image->data = (unsigned char*)malloc(width * height * nrChannels);
    image->is_ptr_owned = true;
    std::memset(image->data, 0, width * height * nrChannels);
    return image;
}

Ref<Image> Image::fromPtr(int width, int height, int nrChannels, unsigned char* ptr) {
    Ref<Image> image = Resources::make<Image>();
    image->width = width;
    image->height = height;
    image->nrChannels = nrChannels;
    image->data = ptr;
    image->is_ptr_owned = false;
    return image;
}

void Image::release() {
    if (data && is_ptr_owned) {
        free(data);
    }
}

void Image::toFilePNG(const std::string& filename) {
    stbi_write_png(filename.c_str(), width, height, nrChannels, data, nrChannels * width);
}
