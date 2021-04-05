//
// Created by lasagnaphil on 19. 3. 14.
//

#ifndef MOTION_EDITING_IMAGE_H
#define MOTION_EDITING_IMAGE_H

#include <string>
#include "gengine/Arena.h"

struct Image {
    unsigned char* data = nullptr;
    int width, height, nrChannels, desiredChannels;

    Image() = default;
    ~Image() { dispose(); }

    static Ref<Image> fromFile(const std::string& filename, int desiredChannels = 0);

    void dispose();
};
#endif //MOTION_EDITING_IMAGE_H
