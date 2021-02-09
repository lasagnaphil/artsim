//
// Created by lasagnaphil on 2/21/18.
//

#ifndef GENGINE_ARENA_H
#define GENGINE_ARENA_H

#include <cstring>
#include <cassert>
#include <utility>
#include <vector>
#include <cstdint>
#include <cstdlib>
#include <optional>

#include "IDisposable.h"

#define USE_SHARED_PTR
#ifdef USE_SHARED_PTR
#include <memory>
template <class T>
using Ref = std::shared_ptr<T>;

struct Resources {
    template <class T, class ...Args>
    static Ref<T> make(Args&&... args) {
        return std::shared_ptr<T>(new T(std::forward<Args>(args)...));
    }
};

#else

#if __cplusplus == 201402L
#define IF_CONSTEXPR if
#else
#define IF_CONSTEXPR if constexpr
#endif

class TypeRegistry {
public:
    template <typename T> static uint16_t getID();
};

template <typename T>
struct Ref {
    uint32_t index;
    uint32_t generation;

    static Ref fromInt64(int64_t data) {
        return *reinterpret_cast<Ref*>(&data);
    }
    int64_t toInt64() {
        return *reinterpret_cast<int64_t*>(this);
    }

    static Ref null() {
        return Ref {};
    }

    bool isNull() const {
        return generation == 0;
    }

    explicit operator bool() const {
        return generation != 0;
    }

    bool operator==(const Ref<T>& other) const {
        return index == other.index && generation == other.generation;
    }
    bool operator!=(const Ref<T>& other) const {
        return !((*this) == other);
    }

    T* operator->() const;
    T& operator*() const;

    T* get();
    T* tryGet();
    void release();
    void reset() {
        release();
    }
};

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
        data = (T*)aligned_alloc(alignof(T), capacity * sizeof(T));
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
#ifdef __APPLE__
        data = (T*)malloc(newCapacity * sizeof(T));
#else
        data = (T*)aligned_alloc(alignof(T), newCapacity * sizeof(T));
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
    Ref<T> make(Args&&... args) {
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
        return Ref<T> {newIndex, newIndices.generation};
    }

    Ref<T> clone(Ref<T> ref) {
        Ref<T> newRef = make();
        data[newRef.index] = data[ref.index];
        return newRef;
    }

    bool has(Ref<T> ref) const {
        auto idx = indices[ref.index];
        return idx.generation != 0 && idx.generation == ref.generation;
    }

    const T* get(Ref<T> ref) const {
        auto idx = indices[ref.index];

        assert(idx.generation != 0);
        assert(idx.generation == ref.generation);

        return &data[ref.index];
    }

    T* get(Ref<T> ref) {
        auto idx = indices[ref.index];

        assert(idx.generation != 0);
        assert(idx.generation == ref.generation);

        return &data[ref.index];
    }

    const T* tryGet(Ref<T> ref) const {
        auto idx = indices[ref.index];

        if (idx.generation != 0 && idx.generation == ref.generation) {
            return &data[ref.index];
        }
        else {
            return nullptr;
        }
    }

    T* tryGet(Ref<T> ref) {
        auto idx = indices[ref.index];

        if (idx.generation != 0 && idx.generation == ref.generation) {
            return &data[ref.index];
        }
        else {
            return nullptr;
        }
    }

    void release(Ref<T> ref) {
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
                Ref<T> ref = {i, indices[i].generation};
                fun(data[i], ref);
            }
        }
    }

    template <class Fun>
    void forEachUntil(Fun&& fun) {
        for (uint32_t i = 0; i < _capacity; ++i) {
            if (indices[i].generation != 0) {
                Ref<T> ref = {i, indices[i].generation};
                bool end = fun(data[i], ref);
                if (end) return;
            }
        }
    }
};

struct Resources {
    friend class constructor;

    template <class T>
    static Arena<T>& getStorage();

    template <class T>
    static T* get(Ref<T> ref) {
        return getStorage<T>().get(ref);
    }

    template <class T>
    static T* tryGet(Ref<T> ref) {
        return getStorage<T>().tryGet(ref);
    }

    template <typename T>
    static Ref<T> clone(Ref<T> ref) {
        return getStorage<T>().clone(ref);
    }

    template <class T, class ...Args>
    static Ref<T> make(Args... args) {
        return getStorage<T>().make(args...);
    }

    template <class T>
    static void release(Ref<T> ref) {
        return getStorage<T>().release(ref);
    }
};
#endif

#endif //GENGINE_ARENA_H
