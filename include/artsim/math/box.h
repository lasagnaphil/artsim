//
// Created by lasagnaphil on 21. 6. 28..
//

#ifndef EOS_SCAN_TO_HUMAN_BOX_H
#define EOS_SCAN_TO_HUMAN_BOX_H

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

namespace glmx {

template <int Dim, class T>
struct tbox {
    glm::vec<Dim, T> lo = glm::vec<Dim, T>(std::numeric_limits<T>::max());
    glm::vec<Dim, T> hi = glm::vec<Dim, T>(-std::numeric_limits<T>::max());

    void extend(const glm::vec<Dim, T>& p) {
        lo = glm::min(lo, p);
        hi = glm::max(hi, p);
    }

    void extend(const glmx::tbox<Dim, T>& box) {
        lo = glm::min(lo, box.lo);
        hi = glm::max(hi, box.hi);
    }

    glm::vec<Dim, T> center() const {
        return (lo + hi) / T(2);
    }
    glm::vec<Dim, T> size() const {
        return hi - lo;
    }
    T volume() const {
        auto s = size();
        return s[0] * s[1] * s[2];
    }

    bool contains(const glm::vec<Dim, T>& p) const {
        for (int i = 0; i < Dim; i++) {
            if (p[i] < lo[i]) return false;
            if (p[i] > hi[i]) return false;
        }
        return true;
    }

    bool collides_with(const tbox<Dim, T>& other) const {
        tbox<Dim, T> min_diff = *this - other;
        return min_diff.contains(glm::vec<Dim, T>(0));
    }
};

template <int Dim, class T>
tbox<Dim, T> operator+(const tbox<Dim, T>& box1, const tbox<Dim, T>& box2) {
    return {box1.lo + box2.lo, box1.hi + box2.hi};
}

template <int Dim, class T>
tbox<Dim, T> operator-(const tbox<Dim, T>& box1, const tbox<Dim, T>& box2) {
    return {box1.lo - box2.hi, box1.hi - box2.lo};
}

template <int Dim, class T>
void distance2(const tbox<Dim, T>& box, const glm::vec<Dim, T>& p) {
    glm::vec<Dim, T> p0 = glm::clamp(p, box.lo, box.hi);
    return glm::distance2(p, p0);
}

template <int Dim, class T>
void distance(const tbox<Dim, T>& box, const glm::vec<Dim, T>& p) {
    return glm::sqrt(distance2(box, p));
}

template <int Dim, class T>
void distance2(const tbox<Dim, T>& box1, const tbox<Dim, T>& box2) {
    auto min_diff = box1 - box2;
    glm::vec<Dim, T> p0 = glm::clamp(glm::vec<Dim, T>(0), min_diff.lo, min_diff.hi);
    return glm::length2(p0);
}

template <int Dim, class T>
void distance(const tbox<Dim, T>& box1, const tbox<Dim, T>& box2) {
    return glm::sqrt(distance2(box1, box2));
}

}


#endif //EOS_SCAN_TO_HUMAN_BOX_H
