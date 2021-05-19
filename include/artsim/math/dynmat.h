//
// Created by lasagnaphil on 20. 10. 17..
//

#ifndef ARTSIM_DYNMAT_H
#define ARTSIM_DYNMAT_H

#include <cstddef>

namespace glmx {

template <class T>
struct dynmat_view {
    T* const ptr;
    uint32_t rows;
    uint32_t cols;
    uint32_t row_stride;
    uint32_t col_stride;

    dynmat_view(T* const ptr, uint32_t rows, uint32_t cols)
    : ptr(ptr), rows(rows), cols(cols), row_stride(rows), col_stride(cols) {}

    dynmat_view(T* const ptr, uint32_t rows, uint32_t cols, uint32_t row_stride, uint32_t col_stride)
    : ptr(ptr), rows(rows), cols(cols), row_stride(row_stride), col_stride(col_stride) {}

    T& operator()(uint32_t i, uint32_t j) {
        return ptr[j*row_stride + i];
    }
    const T& operator()(uint32_t i, uint32_t j) const {
        return ptr[j*row_stride + i];
    }

    dynmat_view<T> slice(uint32_t row_start, uint32_t row_n, uint32_t col_start, uint32_t col_n) {
        return dynmat_view<T>(ptr + col_start*row_stride + row_start, row_n, col_n, row_stride, col_stride);
    }

    void clear_zero() {
        for (int j = 0; j < cols; j++) {
            for (int i = 0; i < rows; i++) {
                ptr[j*row_stride + i] = T(0);
            }
        }
    }
};

template <class T>
struct dynmat {
    T* ptr;
    uint32_t rows;
    uint32_t cols;

    dynmat() : ptr(nullptr), rows(0), cols(0) {}
    dynmat(uint32_t rows, uint32_t cols) : rows(rows), cols(cols) {
        ptr = new T[rows*cols];
    }
    dynmat(const dynmat& m) : rows(m.rows), cols(m.cols) {
        std::copy(m.ptr, m.ptr + rows*cols, ptr);
    }
    dynmat(dynmat&& m) noexcept : ptr(std::move(m.ptr)), rows(std::move(m.rows)), cols(std::move(m.cols)) {}
    dynmat& operator=(const dynmat& m) {
        *this = dynmat(m); return *this;
    }
    dynmat& operator=(dynmat&& m) noexcept {
        std::swap(ptr, m.ptr); std::swap(rows, m.rows); std::swap(cols, m.cols); return *this;
    }
    ~dynmat() { delete[] ptr; }

    dynmat(uint32_t N, glmx::Identity, T value = T(1)) : rows(N), cols(N) {
        ptr = new T[N*N];
        std::fill_n(ptr, N*N, T(0));
        for (int i = 0; i < N; i++) ptr[i*N + i] = value;
    }

    T& operator()(uint32_t i, uint32_t j) {
        return ptr[j*rows + i];
    }
    const T& operator()(uint32_t i, uint32_t j) const {
        return ptr[j*rows + i];
    }

    T* data() { return ptr; }
    const T* data() const { return ptr; }

    dynmat_view<T> slice(uint32_t row_start, uint32_t row_n, uint32_t col_start, uint32_t col_n) {
        return dynmat_view<T>(ptr + col_start*rows + row_start, row_n, col_n, rows, cols);
    }

    dynmat_view<T> to_view() {
        return dynmat_view<T>(ptr, rows, cols, rows, cols);
    }

    void clear_zero() {
        std::fill_n(ptr, rows*cols, T(0));
    }
};

}

#endif //ARTSIM_DYNMAT_H
