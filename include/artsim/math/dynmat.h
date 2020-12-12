//
// Created by lasagnaphil on 20. 10. 17..
//

#ifndef ARTSIM_DYNMAT_H
#define ARTSIM_DYNMAT_H


#include <cstddef>

template <class T>
struct dynmat_view {
    const T* ptr;
    uint32_t rows;
    uint32_t cols;
    uint32_t row_stride;
    uint32_t col_stride;

    dynmat_view(const T* ptr, uint32_t rows, uint32_t cols)
    : ptr(ptr), rows(rows), cols(cols), row_stride(rows), col_stride(cols) {}

    dynmat_view(const T* ptr, uint32_t rows, uint32_t cols, uint32_t row_stride, uint32_t col_stride)
    : ptr(ptr), rows(rows), cols(cols), row_stride(row_stride), col_stride(col_stride) {}

    T& operator()(uint32_t i, uint32_t j) {
        return ptr[i*row_stride + j];
    }
    const T& operator()(uint32_t i, uint32_t j) const {
        return ptr[i*row_stride + j];
    }

    dynmat_view<T> slice(uint32_t row_start, uint32_t row_n, uint32_t col_start, uint32_t col_n) {
        return dynmat_view<T>(ptr + row_start*row_stride + col_start, row_n, col_n, row_stride, col_stride);
    }

    void clear_zero() {
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                ptr[i*row_stride + j] = T(0);
            }
        }
    }
};

template <class T>
struct dynmat {
    T* ptr;
    uint32_t rows;
    uint32_t cols;

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

    T& operator()(uint32_t i, uint32_t j) {
        return ptr[i*rows + j];
    }
    const T& operator()(uint32_t i, uint32_t j) const {
        return ptr[i*rows + j];
    }

    dynmat_view<T> slice(uint32_t row_start, uint32_t row_n, uint32_t col_start, uint32_t col_n) {
        return dynmat_view<T>(ptr + row_start*rows + col_start, row_n, col_n, rows, cols);
    }
};


#endif //ARTSIM_DYNMAT_H
