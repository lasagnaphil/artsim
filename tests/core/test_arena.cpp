//
// Created by lasagnaphil on 3/27/21.
//

#include "doctest.h"

#include <artsim/core/arena.h>

struct alignas(16) Obj {
    int a, b, c, d;
    float x, y;

    Obj(int n = 0) : a(n), b(n), c(n), d(n), x((float)n), y((float)n) {}

    bool operator==(const Obj& other) const {
        return a == other.a && b == other.b && c == other.c && d == other.d
                && x == other.x && y == other.y;
    }

    bool operator!=(const Obj& other) const {
        return !(*this == other);
    }
};

namespace artsim {
DEFINE_TYPEID(Obj, 1)
}

using namespace artsim;

TEST_CASE("Testing Arena") {
    SUBCASE("Simple test") {
        Arena<Obj> arena;

        Id<Obj> a = arena.make(1);
        REQUIRE(arena.size() == 1);
        Id<Obj> b = arena.make(2);
        REQUIRE(arena.size() == 2);
        Id<Obj> c = arena.make();
        *arena.get(c) = 3;
        REQUIRE(arena.size() == 3);

        REQUIRE(*arena.get(a) == 1);
        REQUIRE(*arena.get(b) == 2);
        REQUIRE(*arena.get(c) == 3);
        REQUIRE(arena.is_valid(a));
        REQUIRE(arena.is_valid(b));
        REQUIRE(arena.is_valid(c));

        REQUIRE(!arena.is_valid(Id<Obj>{a.index, 1, static_cast<uint32_t>(a.generation + 1)}));
        REQUIRE(!arena.is_valid(Id<Obj>{a.index, 1, static_cast<uint32_t>(a.generation - 1)}));

        arena.release(a);
        REQUIRE(!arena.is_valid(a));
        REQUIRE(arena.is_valid(b));
        REQUIRE(arena.is_valid(c));
        REQUIRE(arena.size() == 2);

        arena.release(c);
        REQUIRE(!arena.is_valid(a));
        REQUIRE(arena.is_valid(b));
        REQUIRE(!arena.is_valid(c));
        REQUIRE(arena.size() == 1);

        arena.release(b);
        REQUIRE(!arena.is_valid(a));
        REQUIRE(!arena.is_valid(b));
        REQUIRE(!arena.is_valid(c));
        REQUIRE(arena.size() == 0);
    }

    auto shuffle = [](int* arr, size_t n) {
        if (n > 1) {
            size_t i;
            srand(time_t(NULL));
            for (i = 0; i < n - 1; i++) {
                size_t j = i + rand() / (RAND_MAX / (n - i) + 1);
                int t = arr[j];
                arr[j] = arr[i];
                arr[i] = t;
            }
        }
    };

    SUBCASE("Complex test") {
        int testSize = 1024*1024;
        auto arena = Arena<Obj>(testSize);
        std::vector<Id<Obj>> refs(testSize);
        for (int i = 0; i < testSize; i++) {
            refs[i] = arena.make(i);
        }
        REQUIRE(arena.size() == testSize);
        int deleteSize = 1024*1024/2;
        std::vector<int> shuffled(testSize);
        for (int i = 0; i < testSize; i++) {
            shuffled[i] = i;
        }
        shuffle(shuffled.data(), shuffled.size());
        for (int i = 0; i < deleteSize; i++) {
            arena.release(refs[shuffled[i]]);
        }
        REQUIRE(arena.size() == testSize - deleteSize);

        for (int i = 0; i < deleteSize; i++) {
            REQUIRE(!arena.is_valid(refs[shuffled[i]]));
        }
        for (int i = deleteSize; i < testSize; i++) {
            REQUIRE(arena.is_valid(refs[shuffled[i]]));
            REQUIRE(*arena.get(refs[shuffled[i]]) == shuffled[i]);
        }

        std::vector<Id<Obj>> newIds(deleteSize);
        for (int i = 0; i < deleteSize; i++) {
            newIds[i] = arena.make(shuffled[i]);
        }
        REQUIRE(arena.size() == testSize);
        for (int i = 0; i < deleteSize; i++) {
            REQUIRE(arena.is_valid(newIds[i]));
            REQUIRE(*arena.get(newIds[i]) == shuffled[i]);
        }
        for (int i = deleteSize; i < testSize; i++) {
            REQUIRE(arena.is_valid(refs[shuffled[i]]));
            REQUIRE(*arena.get(refs[shuffled[i]]) == shuffled[i]);
        }
    }
}