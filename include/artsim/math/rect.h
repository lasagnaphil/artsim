//
// Created by lasagnaphil on 19. 11. 12..
//

#ifndef GENGINE_RECT_H
#define GENGINE_RECT_H

#include <glm/vec2.hpp>

namespace glmx {
template <class T>
struct trect {
    glm::tvec2<T> min; // upper left
    glm::tvec2<T> max; // lower right

    bool isInside(glm::tvec2<T> p) const {
        return min.x < p.x && p.x < max.x && min.y < p.y && p.y < max.y;
    }
};

template <class T>
struct tbox {
    glm::tvec3<T> min;
    glm::tvec3<T> max;
};

using rect = trect<float>;
using box = tbox<float>;
}

#endif //GENGINE_RECT_H
