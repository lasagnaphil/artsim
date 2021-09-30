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

    image->data = stbi_load(filename.c_str(), &image->width, &image->height, &image->nrChannels, desiredChannels);
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
    return image;
}

void Image::dispose() {
    if (data) {
        free(data);
    }
}

void Image::toFilePNG(const std::string& filename) {
    stbi_write_png(filename.c_str(), width, height, nrChannels, data, nrChannels * width);
}
