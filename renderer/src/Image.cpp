//
// Created by lasagnaphil on 19. 3. 14.
//

#include <stb_image.h>

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

void Image::dispose() {
    if (data) {
        stbi_image_free(data);
    }
}