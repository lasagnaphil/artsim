//
// Created by lasagnaphil on 7/10/18.
//

#ifndef ALTLIB_DEFER_H
#define ALTLIB_DEFER_H


#include <functional>

struct defer_dummy {};
template <class F> struct deferrer { F f; ~deferrer() { f(); } };
template <class F> deferrer<F> operator*(defer_dummy, F f) { return {f}; }
#define DEFER_(LINE) zz_defer##LINE
#define DEFER(LINE) DEFER_(LINE)
#define defer auto DEFER(__LINE__) = defer_dummy{} *[&]()

#endif //ALTLIB_DEFER_H
