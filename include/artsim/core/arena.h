//
// Created by lasagnaphil on 2/21/18.
//

#ifndef MOTION_EDITING_GLOBALSTORAGE_H
#define MOTION_EDITING_GLOBALSTORAGE_H

#include <cstring>
#include <cassert>
#include <utility>
#include <vector>
#include <cstdint>
#include <cstdlib>
#include <optional>

class TypeRegistry {
public:
    template <typename T> static uint16_t getID();
};

template <typename T>
struct Id {
    uint32_t index;
    uint32_t generation;

    static Id fromInt64(int64_t data) {
        return *reinterpret_cast<Id*>(&data);
    }
    int64_t toInt64() {
        return *reinterpret_cast<int64_t*>(this);
    }

    static Id null() {
        return Id {};
    }

    bool isNull() const {
        return generation == 0;
    }

    explicit operator bool() const {
        return generation != 0;
    }

    bool operator==(const Id<T>& other) const {
        return index == other.index && generation == other.generation;
    }
    bool operator!=(const Id<T>& other) const {
        return !((*this) == other);
    }
};

namespace std {
    template <class T>
    struct hash<Id<T>>
    {
        std::size_t operator()(const Id<T>& id) const {
            using std::hash;
            return hash<uint32_t>()(id.index) ^ (hash<uint32_t>()(id.generation) << 1);
        }
    };
}

template <typename T>
struct Arena {

private:
    struct Indices {
        uint32_t nextIndex;
        uint32_t generation;
    };

    // Note: the following doesn't work because of some undefined behavior? Need to examine this later.
    // std::vector<std::aligned_storage_t<sizeof(T), alignof(T)>> data;
    T* data;
    std::vector<Indices> indices;

    uint32_t _size;
    uint32_t _capacity;

    uint32_t firstAvailable;

public:
    Arena(uint32_t capacity = 0) :
            _size(0), _capacity(capacity), data(nullptr), indices(capacity), firstAvailable(0)
    {
#if defined(_WIN64)
        data = (T*)_aligned_malloc(capacity * sizeof(T), alignof(T));
#elif defined(__APPLE__)
        data = (T*)malloc(capacity * sizeof(T));
#else
        data = (T*)std::aligned_alloc(alignof(T), capacity * sizeof(T));
#endif
        for (uint32_t i = 0; i < capacity; ++i) {
            indices[i].nextIndex = i + 1;
            indices[i].generation = 0;
        }
    }

    ~Arena() {
        for (uint32_t i = 0; i < _capacity; ++i) {
            if (indices[i].generation != 0) {
                (data[i]).~T();
            }
        }
    }

    inline uint32_t size() { return _size; }
    inline uint32_t capacity() { return _capacity; }

    void expand(uint32_t newCapacity) {
        assert (newCapacity >= _capacity);

        T* oldData = data;
        // For some strange reason, aligned_alloc sometimes returns NULL on MacOS.
        // malloc() in MacOS is always 16-byte aligned, so let's just use it instead.

#if defined(_WIN64)
        data = (T*)_aligned_malloc(newCapacity* sizeof(T), alignof(T));
#elif defined(__APPLE__)
        data = (T*)malloc(newCapacity * sizeof(T));
#else
        data = (T*)std::aligned_alloc(alignof(T), newCapacity * sizeof(T));
#endif
        indices.resize(newCapacity);

        // invoke move constructor for filled items
        for (uint32_t i = 0; i < _capacity; i++) {
            if (indices[i].generation != 0) {
                new (data + i) T(std::move(oldData[i]));
            }
        }
        // construct the free list of the indices
        for (uint32_t i = _capacity; i < newCapacity; ++i) {
            indices[i].nextIndex = i + 1;
            indices[i].generation = 0;
        }

        _capacity = newCapacity;

        free(oldData);
    }

    template <class ...Args>
    Id<T> make(Args&&... args) {
        // if the item list is full
        if (firstAvailable == _capacity) {
            expand(_capacity == 0 ? 4 : _capacity * 2);
        }

        // delete node from free list
        uint32_t newIndex = firstAvailable;
        new(data + newIndex) T(std::forward<Args>(args)...);
        auto& newIndices = indices[newIndex];
        firstAvailable = newIndices.nextIndex;

        newIndices.generation++;

        _size++;

        // also return the reference object of the resource
        return Id<T> {newIndex, newIndices.generation};
    }

    Id<T> clone(Id<T> ref) {
        Id<T> newId = make();
        data[newId.index] = data[ref.index];
        return newId;
    }

    bool has(Id<T> ref) const {
        auto idx = indices[ref.index];
        return idx.generation != 0 && idx.generation == ref.generation;
    }

    const T* get(Id<T> ref) const {
        auto idx = indices[ref.index];

        assert(idx.generation != 0);
        assert(idx.generation == ref.generation);

        return &data[ref.index];
    }

    T* get(Id<T> ref) {
        auto idx = indices[ref.index];

        assert(idx.generation != 0);
        assert(idx.generation == ref.generation);

        return &data[ref.index];
    }

    const T* tryGet(Id<T> ref) const {
        auto idx = indices[ref.index];

        if (idx.generation != 0 && idx.generation == ref.generation) {
            return &data[ref.index];
        }
        else {
            return nullptr;
        }
    }

    T* tryGet(Id<T> ref) {
        auto idx = indices[ref.index];

        if (idx.generation != 0 && idx.generation == ref.generation) {
            return &data[ref.index];
        }
        else {
            return nullptr;
        }
    }

    void release(Id<T> ref) {
        auto idx = indices[ref.index];
        assert(idx.generation != 0);
        assert(idx.generation == ref.generation);

        indices[ref.index].nextIndex = firstAvailable;
        indices[ref.index].generation = 0;
        firstAvailable = ref.index;

        _size--;
    }

    template <class Fun>
    void forEach(Fun&& fun) {
        for (uint32_t i = 0; i < _capacity; ++i) {
            if (indices[i].generation != 0) {
                Id<T> ref = {i, indices[i].generation};
                fun(data[i], ref);
            }
        }
    }

    template <class Fun>
    void forEachUntil(Fun&& fun) {
        for (uint32_t i = 0; i < _capacity; ++i) {
            if (indices[i].generation != 0) {
                Id<T> ref = {i, indices[i].generation};
                bool end = fun(data[i], ref);
                if (end) return;
            }
        }
    }
};

#endif //MOTION_EDITING_GLOBALSTORAGE_H
