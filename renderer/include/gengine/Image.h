//
// Created by lasagnaphil on 19. 3. 14.
//

#ifndef MOTION_EDITING_IMAGE_H
#define MOTION_EDITING_IMAGE_H

#include <string>
#include "gengine/Arena.h"

struct Image {
    unsigned char* data = nullptr;
    enum class Type {
        Owned, View, Mapped
    };
    bool is_ptr_owned = false;
    int width, height, nrChannels, desiredChannels;
    std::string filename;

    Image() = default;
    ~Image() { release(); }
    Image(const Image& other) :
        width(other.width), height(other.height),
        nrChannels(other.nrChannels), desiredChannels(other.desiredChannels), is_ptr_owned(other.is_ptr_owned) {

        if (other.is_ptr_owned) {
            data = (unsigned char*)malloc(width * height * nrChannels);
            std::memcpy(data, other.data, width * height * nrChannels);
        }
        else {
            data = other.data;
        }
    }
    Image(Image&& other) noexcept :
        width(other.width), height(other.height),
        nrChannels(other.nrChannels), desiredChannels(other.desiredChannels),
        is_ptr_owned(other.is_ptr_owned), data(std::exchange(other.data, nullptr)) {}

    Image& operator=(const Image& other) {
        return *this = Image(other);
    }

    Image& operator=(Image&& other) noexcept {
        std::swap(width, other.width);
        std::swap(height, other.height);
        std::swap(nrChannels, other.nrChannels);
        std::swap(desiredChannels, other.desiredChannels);
        std::swap(data, other.data);
        return *this;
    }

    static Ref<Image> fromFile(const std::string& filename, int desiredChannels = 0);
    static Ref<Image> fromEmpty(int width, int height, int nrChannels);
    static Ref<Image> fromPtr(int width, int height, int nrChannels, unsigned char* ptr);

    void toFilePNG(const std::string& filename);

    void release();
};
#endif //MOTION_EDITING_IMAGE_H
