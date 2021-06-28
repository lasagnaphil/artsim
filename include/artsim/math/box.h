//
// Created by lasagnaphil on 21. 6. 28..
//

#ifndef EOS_SCAN_TO_HUMAN_BOX_H
#define EOS_SCAN_TO_HUMAN_BOX_H

namespace glmx {

template <int Dim, class T>
struct tbox {
    glm::vec<Dim, T> lo = glm::vec<Dim, T>(std::numeric_limits<T>::max);
    glm::vec<Dim, T> hi = glm::vec<Dim, T>(std::numeric_limits<T>::min);

    void extend(glm::vec<Dim, T> p) {
        for (int i = 0; i < Dim; i++) {
            if (p < lo[i]) lo[i] = p;
            if (p > hi[i]) hi[i] = p;
        }
    }

    glm::vec<Dim, T> center() {
        return (lo + hi) / 2;
    }
    glm::vec<Dim, T> size() {
        return hi - lo;
    }
    real volume() {
        auto s = size();
        return s[0] * s[1] * s[2];
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
