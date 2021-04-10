//
// Created by lasagnaphil on 2/21/18.
//

#ifndef MOTION_EDITING_GLOBALSTORAGE_H
#define MOTION_EDITING_GLOBALSTORAGE_H

// Implementation of generational arena based on:
// https://www.gamedev.net/tutorials/programming/general-and-gameplay-programming/game-engine-containers-handle_map-r4495/

#include <cstring>
#include <cassert>
#include <utility>
#include <vector>
#include <cstdint>
#include <cstdlib>
#include <optional>

namespace artsim {

template <class T>
struct TypeID {
    uint32_t operator()() const { return 0; }
};

#define DEFINE_TYPEID(T, id) \
template <> struct TypeID<T> { uint32_t operator()() const { return id; } };

template <typename T>
struct Id {
    uint32_t index;
    uint32_t type : 8;
    uint32_t generation : 24;

    static Id from_int64(int64_t data) {
        return *reinterpret_cast<Id*>(&data);
    }
    int64_t to_int64() {
        return *reinterpret_cast<int64_t*>(this);
    }
    static Id from_int32s(uint32_t id1, uint32_t id2) {
        Id id;
        uint32_t* ptr = reinterpret_cast<uint32_t*>(&id);
        ptr[0] = id1;
        ptr[1] = id2;
        return id;
    }
    std::pair<int32_t, int32_t> to_int32s() {
        uint32_t* ptr = reinterpret_cast<uint32_t*>(this);
        return {ptr[0], ptr[1]};
    }

    static Id null() {
        return Id {0, TypeID<T>()(), 1};
    }

    bool is_null() const {
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

struct AnyId : public Id<void> {};


template <typename T>
class Arena {
private:
    std::vector<T> items;
    std::vector<Id<T>> free_list;
    std::vector<uint32_t> dense_to_sparse_map;

    uint32_t free_list_front = 0xFFFFFFFF;
    uint32_t free_list_back = 0xFFFFFFFF;
    bool fragmented = false;

public:
    Arena(uint32_t capacity = 0)
    {
        items.reserve(capacity);
        free_list.reserve(capacity);
        dense_to_sparse_map.reserve(capacity);
    }

    uint32_t size() const { return items.size(); }
    uint32_t capacity() const { return items.capacity(); }
    typename std::vector<T>::iterator begin() { return items.begin(); }
    typename std::vector<T>::const_iterator cbegin() const { return items.cbegin(); }
    typename std::vector<T>::iterator end() { return items.end(); }
    typename std::vector<T>::const_iterator cend() const { return items.cend(); }
    T* get_items_buf() { return items.data(); }
    const T* get_items_buf() const { return items.data(); }

    void clear() {
        items.clear();
        free_list.clear();
        dense_to_sparse_map.clear();
        free_list_front = free_list_back = 0xFFFFFFFF;
        fragmented = false;
    }

    Id<T> insert(T&& item) {
        Id<T> id;
        if (free_list_front == 0xFFFFFFFF) {
            // If free list is empty, create new free list node
            Id<T> node = {(uint32_t)items.size(), TypeID<T>()(), 1};
            id = {(uint32_t)free_list.size(), TypeID<T>()(), 1};
            free_list.push_back(node);
        }
        else {
            // Take the front node of free list
            Id<T>& node = free_list[free_list_front];
            uint32_t new_index = free_list_front;
            free_list_front = node.index;
            if (free_list_front == 0xFFFFFFFF) {
                free_list_back = free_list_front;
            }
            node.index = items.size();
            id = {new_index, TypeID<T>()(), node.generation};
        }
        items.push_back(item);
        dense_to_sparse_map.push_back(id.index);
        return id;
    }

    Id<T> insert(const T& item) {
        return insert(std::move(T(item)));
    }

    template <class ...Args>
    Id<T> make(Args&&... args) {
        return insert(T(std::forward<Args>(args)...));
    }

    template <class ...Args>
    Id<T> emplace(Args&&... args) {
        return insert(T(std::forward<Args>(args)...));
    }

    template <class ...Args>
    void emplace_n(int n, Id<T>* ids, Args&&... args) {
        items.reserve(items.size() + n);
        dense_to_sparse_map.reserve(dense_to_sparse_map.size() + n);
        fragmented = true;

        for (int i = 0; i < n; i++) {
            ids[i] = emplace(args...);
        }
    }

    bool release(Id<T> id) {
        return erase(id);
    }

    bool erase(Id<T> id) {
        if (!is_valid(id)) return false;
        auto& node = free_list[id.index];

        uint32_t prev_index = node.index;
        fragmented = true;
        node.index = 0xFFFFFFFF;
        node.generation++;

        if (free_list_front == 0xFFFFFFFF) {
            free_list_back = free_list_front = id.index;
        }
        else {
            free_list[free_list_back].index = id.index;
            free_list_back = id.index;
        }

        if (prev_index != items.size() - 1) {
            std::swap(items.at(prev_index), items.back());
            std::swap(dense_to_sparse_map.at(prev_index), dense_to_sparse_map.back());

            free_list[dense_to_sparse_map.at(prev_index)].index = prev_index;
        }

        items.pop_back();
        dense_to_sparse_map.pop_back();
        return true;
    }

    Id<T> clone(Id<T> id) {
        Id<T> newId = insert(T());
        T* data_ptr = get(newId);
        *data_ptr = items[id.index];
        return newId;
    }

    bool is_valid(Id<T> id) const {
        if (id.index >= free_list.size()) return false;
        auto node = free_list[id.index];
        return node.index < size() && node.generation != 0 && node.generation == id.generation;
    }

    const T* get(Id<T> id) const {
        assert(id.index < free_list.size());
        auto node = free_list[id.index];
        assert(node.generation != 0);
        assert(node.generation == id.generation);
        assert(node.index < size());

        return items.data() + node.index;
    }

    T* get(Id<T> id) {
        assert(id.index < free_list.size());
        auto node = free_list[id.index];
        assert(node.generation != 0);
        assert(node.generation == id.generation);
        assert(node.index < size());

        return items.data() + node.index;
    }

    const T* try_get(Id<T> id) const {
        auto node = free_list[id.index];

        if (node.generation != 0 && node.generation == id.generation) {
            return items.data() + node.index;
        }
        else {
            return nullptr;
        }
    }

    T* try_get(Id<T> id) {
        auto node = free_list[id.index];

        if (node.generation != 0 && node.generation == id.generation) {
            return items.data() + node.index;
        }
        else {
            return nullptr;
        }
    }

    Id<T> get_id_of_ptr(T* data_ptr) {
        int item_idx = data_ptr - items.data();
        int node_idx = dense_to_sparse_map[item_idx];
        auto& node = free_list[node_idx];
        assert(node.generation != 0);
        return Id<T> {node.index, TypeID<T>()(), node.generation};
    }

};

}

namespace std {
template <class T>
struct hash<artsim::Id<T>>
{
    std::size_t operator()(const artsim::Id<T>& id) const {
        using std::hash;
        return hash<uint32_t>()(id.index) ^ (hash<uint32_t>()(id.generation) << 1);
    }
};
}


#endif //MOTION_EDITING_GLOBALSTORAGE_H
