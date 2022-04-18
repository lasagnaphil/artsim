//
// Created by lasagnaphil on 20. 12. 11..
//

#ifndef ARTLIB_DYNVEC_H
#define ARTLIB_DYNVEC_H

#include <cstddef>
#include <memory>
#include <algorithm>

namespace glmx {

template <class T>
struct dynvec_view {
    T* const ptr;
    uint32_t rows;

    dynvec_view(T* const ptr, uint32_t rows)
            : ptr(ptr), rows(rows) {}

    T& operator()(uint32_t i) {
        return ptr[i];
    }
    const T& operator()(uint32_t i) const {
        return ptr[i];
    }

    dynvec_view<T> slice(uint32_t row_start, uint32_t row_n) {
        return dynvec_view<T>(ptr + row_start, row_n);
    }

    void clear_zero() {
        std::fill(ptr, ptr + rows, 0);
    }

    template <class Fun>
    void iterate(Fun&& fun) {
        for (int i = 0; i < rows; i++) {
            fun(ptr[i]);
        }
    }
};

template <class T>
struct dynvec {
    T* ptr;
    uint32_t size;
    uint32_t capacity;

    dynvec(uint32_t size) : size(size), capacity(size) {
        ptr = new T[capacity];
    }
    dynvec(uint32_t size, T elem) : size(size), capacity(size) {
        ptr = new T[capacity];
        std::fill_n(ptr, size, elem);
    }
    dynvec(const dynvec& m) : size(m.size), capacity(m.size) {
        ptr = new T[capacity];
        std::copy(m.ptr, m.ptr + size, ptr);
    }
    dynvec(dynvec&& m) noexcept : ptr(std::move(m.ptr)), size(std::move(m.size)) {}
    dynvec& operator=(const dynvec& m) {
        *this = dynvec(m); return *this;
    }
    dynvec& operator=(dynvec&& m) noexcept {
        std::swap(ptr, m.ptr); std::swap(size, m.size); return *this;
    }
    ~dynvec() { delete[] ptr; }

    T& operator()(uint32_t i) {
        return ptr[i];
    }
    const T& operator()(uint32_t i) const {
        return ptr[i];
    }

    dynvec_view<T> slice(uint32_t row_start, uint32_t row_n) {
        return dynvec_view<T>(ptr + row_start, row_n);
    }

    void expand(uint32_t new_capacity) {
        assert(new_capacity >= capacity);
        T* new_ptr = new T[new_capacity];
        std::copy_n(ptr, capacity, new_ptr);
        delete ptr;
        ptr = new_ptr;
    }

    void push_back(T elem) {
        if (size == capacity) {
            expand(capacity * 2);
        }
        ptr[size] = elem;
        size++;
    }

    template <class Fun>
    void iterate(Fun&& fun) {
        for (int i = 0; i < size; i++) {
            fun(ptr[i]);
        }
    }
};

}
#endif //ARTLIB_DYNVEC_H