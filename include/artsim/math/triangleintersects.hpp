#ifndef TRIANGLEINTERSECTS_H
#define TRIANGLEINTERSECTS_H

#include <utility>
#include <type_traits>
#include <iostream>
#include <cmath>
#include <glm/vec3.hpp>

/**
 * Single header mplementation of Triangle-Triangle and Triangle-Box intersection tests by Tomas Akenine Moeller
 * @see http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/tribox3.txt
 * @see http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/tritri_isectline.txt
 */

namespace artsim {

template<class T>
class TriangleIntersects {
    using Triangle = std::array<glm::tvec3<T>, 3>;

public:
    // Magic number from orginal Moeller's implementation:
    // distances smaller than EPSILON may be treated as ZERO
    static constexpr T EPSILON = T(1e-6);

    static constexpr T epsilon() { return EPSILON; }

    // returns true iff two triangles are intersecting or touching
    static bool triangle(const glm::tvec3<T>& firstV1, const glm::tvec3<T>& firstV2, const glm::tvec3<T>& firstV3,
                         const glm::tvec3<T>& secondV1, const glm::tvec3<T>& secondV2,
                         const glm::tvec3<T>& secondV3) {
        glm::tvec3<T> IntersectionLineEndPoint1;
        glm::tvec3<T> IntersectionLineEndPoint2;
        bool coplanar;
        double debug_reduced_to_0__;  // to be removed once the code has been debugged && understood
        return tri_tri_intersect_with_isectline(firstV1, firstV2, firstV3, secondV1, secondV2, secondV3, coplanar,
                                                IntersectionLineEndPoint1, IntersectionLineEndPoint2, false,
                                                debug_reduced_to_0__);
    }

    // similar to the previous function, but returns additional information
    // on the intersection line in the case of non-parllel triangles
    // and whether the two triangles are coplanar (in which case the intersection line is undefined)
    static bool triangle(const glm::tvec3<T>& firstV1, const glm::tvec3<T>& firstV2, const glm::tvec3<T>& firstV3,
                         const glm::tvec3<T>& secondV1, const glm::tvec3<T>& secondV2, const glm::tvec3<T>& secondV3,
                         glm::tvec3<T>& IntersectionLineEndPoint1, glm::tvec3<T>& IntersectionLineEndPoint2,
                         bool& coplanar) {
        double debug_reduced_to_0__ = 0.0;  // to be removed once the code has been debugged && understood
        return tri_tri_intersect_with_isectline(firstV1, firstV2, firstV3, secondV1, secondV2, secondV3, coplanar,
                                                IntersectionLineEndPoint1, IntersectionLineEndPoint2, true,
                                                debug_reduced_to_0__);
    }

    // tests if a triangle intersects a box with its faces parallel to the x-y-z Cartesian axis
    static bool box(const glm::tvec3<T> triangleV1, const glm::tvec3<T> triangleV2, const glm::tvec3<T> triangleV3,
                    const glm::tvec3<T> boxCenter, const glm::tvec3<T> boxHalfSize) {
        return triBoxOverlap(triangleV1, triangleV2, triangleV3, boxCenter, boxHalfSize);
    }

private:
    // Constants definitions
    static constexpr size_t X = 0;
    static constexpr size_t Y = 1;
    static constexpr size_t Z = 2;

    // Helper methods

    // cross product
    inline static void cross(glm::tvec3<T>& dest, const glm::tvec3<T>& v1, const glm::tvec3<T>& v2) {
        dest[X] = v1[Y] * v2[Z] - v1[Z] * v2[Y];
        dest[Y] = v1[Z] * v2[X] - v1[X] * v2[Z];
        dest[Z] = v1[X] * v2[Y] - v1[Y] * v2[X];
    }

    // dot product
    inline static T dot(const glm::tvec3<T>& v1, const glm::tvec3<T>& v2) {
        return v1[X] * v2[X] + v1[Y] * v2[Y] + v1[Z] * v2[Z];
    }

    // vector normalization
    inline static void normalize(glm::tvec3<T>& v) {
        constexpr double zero_length_threshold = 1e-20;
        double norm = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (norm > zero_length_threshold) {
            v[0] /= norm;
            v[1] /= norm;
            v[2] /= norm;
        } else {
            throw std::logic_error("cannot normalize a zero-length vector");
        }
    }

    // vector subtraction
    inline static void sub(glm::tvec3<T>& dest, const glm::tvec3<T>& v1, const glm::tvec3<T>& v2) {
        dest[X] = v1[X] - v2[X];
        dest[Y] = v1[Y] - v2[Y];
        dest[Z] = v1[Z] - v2[Z];
    }

    // vector addition
    inline static void add(glm::tvec3<T>& dest, const glm::tvec3<T>& v1, const glm::tvec3<T>& v2) {
        dest[X] = v1[X] + v2[X];
        dest[Y] = v1[Y] + v2[Y];
        dest[Z] = v1[Z] + v2[Z];
    }

    // vector product by scalar
    inline static void mult(glm::tvec3<T>& dest, const glm::tvec3<T>& v, const T factor) {
        dest[X] = factor * v[X];
        dest[Y] = factor * v[Y];
        dest[Z] = factor * v[Z];
    }

    // assignment
    inline static void set(glm::tvec3<T>& dest, const glm::tvec3<T>& src) {
        dest[X] = src[X];
        dest[Y] = src[Y];
        dest[Z] = src[Z];
    }

    inline static void sort(T& a, T& b) {
        if (a > b) {
            std::swap(a, b);
        }
    }

    inline static bool sort2(T& a, T& b) {
        if (a > b) {
            std::swap(a, b);
            return true;
        }
        return false;
    }

    inline static void findMinMax(const T x0, const T x1, const T x2, T& min,
                                  T& max) {
        min = std::min(std::min(x0, x1), x2);
        max = std::max(std::max(x0, x1), x2);
    }

    // Tests for Box-Triangle
    inline static bool
    axisTestX01(const glm::tvec3<T>& v0, const glm::tvec3<T>& v2, const glm::tvec3<T>& boxhalfsize,
                const T a, const T b, const T fa, const T fb,
                T& min, T& max, T& rad) {
        T p0 = a * v0[Y] - b * v0[Z];
        T p2 = a * v2[Y] - b * v2[Z];
        if (p0 < p2) {
            min = p0;
            max = p2;
        } else {
            min = p2;
            max = p0;
        }
        rad = fa * boxhalfsize[Y] + fb * boxhalfsize[Z];
        if (min > rad || max < -rad) {
            return false;
        }
        return true;
    }

    inline static bool axisTestX2(const glm::tvec3<T>& v0, const glm::tvec3<T>& v1, const glm::tvec3<T>& boxhalfsize,
                                  const T a, const T b, const T fa, const T fb,
                                  T& min, T& max, T& rad) {
        T p0 = a * v0[Y] - b * v0[Z];
        T p1 = a * v1[Y] - b * v1[Z];
        if (p0 < p1) {
            min = p0;
            max = p1;
        } else {
            min = p1;
            max = p0;
        }
        rad = fa * boxhalfsize[Y] + fb * boxhalfsize[Z];
        if (min > rad || max < -rad) {
            return false;
        }
        return true;
    }

    inline static bool
    axisTestY02(const glm::tvec3<T>& v0, const glm::tvec3<T>& v2, const glm::tvec3<T>& boxhalfsize,
                const T a, const T b, const T fa, const T fb,
                T& min, T& max, T& rad) {
        T p0 = -a * v0[X] + b * v0[Z];
        T p2 = -a * v2[X] + b * v2[Z];
        if (p0 < p2) {
            min = p0;
            max = p2;
        } else {
            min = p2;
            max = p0;
        }
        rad = fa * boxhalfsize[X] + fb * boxhalfsize[Z];
        if (min > rad || max < -rad) {
            return false;
        }
        return true;
    }

    inline static bool axisTestY1(const glm::tvec3<T>& v0, const glm::tvec3<T>& v1, const glm::tvec3<T>& boxhalfsize,
                                  const T a, const T b, const T fa, const T fb,
                                  T& min, T& max, T& rad) {
        T p0 = -a * v0[X] + b * v0[Z];
        T p1 = -a * v1[X] + b * v1[Z];
        if (p0 < p1) {
            min = p0;
            max = p1;
        } else {
            min = p1;
            max = p0;
        }
        rad = fa * boxhalfsize[X] + fb * boxhalfsize[Z];
        if (min > rad || max < -rad) {
            return false;
        }
        return true;
    }

    inline static bool
    axisTestZ12(const glm::tvec3<T>& v1, const glm::tvec3<T>& v2, const glm::tvec3<T>& boxhalfsize,
                const T a, const T b, const T fa, const T fb,
                T& min, T& max, T& rad) {
        T p1 = a * v1[X] - b * v1[Y];
        T p2 = a * v2[X] - b * v2[Y];
        if (p2 < p1) {
            min = p2;
            max = p1;
        } else {
            min = p1;
            max = p2;
        }
        rad = fa * boxhalfsize[X] + fb * boxhalfsize[Y];
        if (min > rad || max < -rad) {
            return false;
        }
        return true;
    }

    inline static bool axisTestZ0(const glm::tvec3<T>& v0, const glm::tvec3<T>& v1, const glm::tvec3<T>& boxhalfsize,
                                  const T a, const T b, const T fa, const T fb,
                                  T& min, T& max, T& rad) {
        T p0 = a * v0[X] - b * v0[Y];
        T p1 = a * v1[X] - b * v1[Y];
        if (p0 < p1) {
            min = p0;
            max = p1;
        } else {
            min = p1;
            max = p0;
        }
        rad = fa * boxhalfsize[X] + fb * boxhalfsize[Y];
        if (min > rad || max < -rad) {
            return false;
        }
        return true;
    }

    // Tests for Triangle-Triangle

    // Tests whether edge U0, U1 intersects with the edge whose origin is V0 and the coordinates
    // of the vector pointing towards the second vertex are Ax (along i0 axis) and Ay (along i1 axis).
    // A = V1 - V0
    // i0 stands for the "x" axis and "i1" for the "y" axis in the new (local) coordinate system.
    // The vertices are projected onto the i0-i1 plane before the actual intersection detection is performed
    //
    // Caveat! This test most likely fails if the two edges ar colinear
    // But this doesn't disturb the final result (other tests will detect the intersection)
    inline static bool edge_edge_test(const glm::tvec3<T>& V0, const glm::tvec3<T>& U0, const glm::tvec3<T>& U1,
                                      const size_t i0, const size_t i1, T Ax, T Ay) {
        T Bx, By, Cx, Cy, f, d, e;
        Bx = U0[i0] - U1[i0];  // B = U0 - U1 (projected onto the i0-i1 plane)
        By = U0[i1] - U1[i1];
        Cx = V0[i0] - U0[i0];  // C = V0 - U0 (projected onto the i0-i1 plane)
        Cy = V0[i1] - U0[i1];
        // if the edges intersect, |f| is half the area of the convex quadrilateral spanned by the vertices
        f = Ay * Bx - Ax * By;
        // if the edges intersect, |d| is half the area of the triangle U0, U1, V0
        d = By * Cx - Bx * Cy;
        if ((f > 0 && d >= 0 && d <= f) || (f < 0 && d <= 0 && d >= f)) {
            // if the edges intersect, |e| is half the area of the triangle V0, V1, U0
            e = Ax * Cy - Ay * Cx;
            if (f > 0) {
                if (e >= 0 && e <= f) return true;
            } else {
                if (e <= 0 && e >= f) return true;
            }
        }
        // all Vertices are colinear iff f == 0 and d == 0, but for some reason this is not tested here
        return false;
    }

    // V0, V1 define an edge
    // U0, U1, U2 define a triangle
    // i0, i1 \in {0,1,2} define the plane (x-y, x-z or y-z) onto which all five vertices are projected
    inline static bool
    edge_against_tri_edge(const glm::tvec3<T>& V0, const glm::tvec3<T>& V1, const glm::tvec3<T>& U0,
                          const glm::tvec3<T>& U1, const glm::tvec3<T>& U2, const size_t i0,
                          const size_t i1) {
        T Ax, Ay;  // coordinates of the edge relative to  V0

        Ax = V1[i0] - V0[i0];
        Ay = V1[i1] - V0[i1];
        /* test intersection of edge U0, U1 with edge V0, V1 */
        if (edge_edge_test(V0, U0, U1, i0, i1, Ax, Ay)) {
            return true;
        }
        /* test edge U1,U2 against V1 - V0 */
        if (edge_edge_test(V0, U1, U2, i0, i1, Ax, Ay)) {
            return true;
        }
        /* test edge U2,U1 against V1 - V0 */
        if (edge_edge_test(V0, U2, U0, i0, i1, Ax, Ay)) {
            return true;
        }
        return false;
    }

    inline static void isect2(const glm::tvec3<T>& VTX0, const glm::tvec3<T>& VTX1, const glm::tvec3<T>& VTX2,
                              const T VV0, const T VV1, const T VV2, const T D0,
                              const T D1, const T D2, T& isect0, T& isect1,
                              glm::tvec3<T>& isectpoint0, glm::tvec3<T>& isectpoint1) {
        T tmp = D0 / (D0 - D1);
        glm::tvec3<T> diff;
        isect0 = VV0 + (VV1 - VV0) * tmp;
        sub(diff, VTX1, VTX0);
        mult(diff, diff, tmp);
        add(isectpoint0, diff, VTX0);
        tmp = D0 / (D0 - D2);
        isect1 = VV0 + (VV2 - VV0) * tmp;
        sub(diff, VTX2, VTX0);
        mult(diff, diff, tmp);
        add(isectpoint1, VTX0, diff);
    }

    inline static bool point_in_tri(const glm::tvec3<T>& V0, const glm::tvec3<T>& U0, const glm::tvec3<T>& U1,
                                    const glm::tvec3<T>& U2, const size_t i0, const size_t i1) {
        T a, b, c, d0, d1, d2;
        /* is T1 completly inside T2? */
        /* check if V0 is inside tri(U0,U1,U2) */
        a = U1[i1] - U0[i1];
        b = -(U1[i0] - U0[i0]);
        c = -a * U0[i0] - b * U0[i1];
        d0 = a * V0[i0] + b * V0[i1] + c;

        a = U2[i1] - U1[i1];
        b = -(U2[i0] - U1[i0]);
        c = -a * U1[i0] - b * U1[i1];
        d1 = a * V0[i0] + b * V0[i1] + c;

        a = U0[i1] - U2[i1];
        b = -(U0[i0] - U2[i0]);
        c = -a * U2[i0] - b * U2[i1];
        d2 = a * V0[i0] + b * V0[i1] + c;

        if (d0 * d1 > 0.0) {
            if (d0 * d2 > 0.0) return true;
        }
        return false;
    }

    // Private methods
    static bool planeBoxOverlap(const glm::tvec3<T>& normal, const glm::tvec3<T>& vert,
                                const glm::tvec3<T>& maxbox)  // -NJMP-
    {
        size_t q;
        T v;
        glm::tvec3<T> vmin, vmax;
        for (q = X; q <= Z; q++) {
            v = vert[q];
            if (normal[q] > 0.0f) {
                vmin[q] = -maxbox[q] - v;
                vmax[q] = maxbox[q] - v;
            } else {
                vmin[q] = maxbox[q] - v;
                vmax[q] = -maxbox[q] - v;
            }
        }
        if (dot(normal, vmin) > 0.0f) {
            return false;
        }
        if (dot(normal, vmax) >= 0.0f) {
            return true;
        }
        return false;
    }

    static bool
    triBoxOverlap(const glm::tvec3<T>& trivert0, const glm::tvec3<T>& trivert1, const glm::tvec3<T>& trivert2,
                  const glm::tvec3<T>& boxcenter, const glm::tvec3<T>& boxhalfsize) {
        /*    use separating axis theorem to test overlap between triangle and box */
        /*    need to test for overlap in these directions: */
        /*    1) the {x,y,z}-directions (actually, since we use the AABB of the triangle */
        /*       we do not even need to test these) */
        /*    2) normal of the triangle */
        /*    3) crossproduct(edge from tri, {x,y,z}-directin) */
        /*       this gives 3x3=9 more tests */

        glm::tvec3<T> v0, v1, v2;
        T min, max, rad, fex, fey, fez;
        glm::tvec3<T> normal, e0, e1, e2;
        /* This is the fastest branch on Sun */
        /* move everything so that the boxcenter is in (0,0,0) */
        sub(v0, trivert0, boxcenter);
        sub(v1, trivert1, boxcenter);
        sub(v2, trivert2, boxcenter);
        /* compute triangle edges */
        sub(e0, v1, v0); /* tri edge 0 */
        sub(e1, v2, v1); /* tri edge 1 */
        sub(e2, v0, v2); /* tri edge 2 */

        /* Bullet 3:  */
        /*  test the 9 tests first (this was faster) */
        fex = fabsf(e0[X]);
        fey = fabsf(e0[Y]);
        fez = fabsf(e0[Z]);

        if (!axisTestX01(v0, v2, boxhalfsize, e0[Z], e0[Y], fez, fey, min, max, rad)) {
            return false;
        }
        if (!axisTestY02(v0, v2, boxhalfsize, e0[Z], e0[X], fez, fex, min, max, rad)) {
            return false;
        }
        if (!axisTestZ12(v1, v2, boxhalfsize, e0[Y], e0[X], fey, fex, min, max, rad)) {
            return false;
        }

        fex = fabsf(e1[X]);
        fey = fabsf(e1[Y]);
        fez = fabsf(e1[Z]);

        if (!axisTestX01(v0, v2, boxhalfsize, e1[Z], e1[Y], fez, fey, min, max, rad)) {
            return false;
        }
        if (!axisTestY02(v0, v2, boxhalfsize, e1[Z], e1[X], fez, fex, min, max, rad)) {
            return false;
        }
        if (!axisTestZ0(v0, v1, boxhalfsize, e1[Y], e1[X], fey, fex, min, max, rad)) {
            return false;
        }

        fex = fabsf(e2[X]);
        fey = fabsf(e2[Y]);
        fez = fabsf(e2[Z]);

        if (!axisTestX2(v0, v1, boxhalfsize, e2[Z], e2[Y], fez, fey, min, max, rad)) {
            return false;
        }
        if (!axisTestY1(v0, v1, boxhalfsize, e2[Z], e2[X], fez, fex, min, max, rad)) {
            return false;
        }
        if (!axisTestZ12(v1, v2, boxhalfsize, e2[Y], e2[X], fey, fex, min, max, rad)) {
            return false;
        }

        /* Bullet 1: */
        /*  first test overlap in the {x,y,z}-directions */
        /*  find min, max of the triangle each direction, and test for overlap in */
        /*  that direction -- this is equivalent to testing a minimal AABB around */
        /*  the triangle against the AABB */
        /* test in X-direction */

        findMinMax(v0[X], v1[X], v2[X], min, max);
        if (min > boxhalfsize[X] || max < -boxhalfsize[X]) {
            return false;
        }
        /* test in Y-direction */
        findMinMax(v0[Y], v1[Y], v2[Y], min, max);
        if (min > boxhalfsize[Y] || max < -boxhalfsize[Y]) {
            return false;
        }
        /* test in Z-direction */
        findMinMax(v0[Z], v1[Z], v2[Z], min, max);
        if (min > boxhalfsize[Z] || max < -boxhalfsize[Z]) {
            return false;
        }

        /* Bullet 2: */
        /*  test if the box intersects the plane of the triangle */
        /*  compute plane equation of triangle: normal*x+d=0 */
        cross(normal, e0, e1);

        if (!planeBoxOverlap(normal, v0, boxhalfsize)) {
            return false;
        }
        return true; /* box and triangle overlaps */
    }

public:
    // the last argument, "debug_reduced_to_0__", is here for research/diagnostic purpose only
    // and most likely will be removed in the final version
    static bool
    tri_tri_intersect_with_isectline(const glm::tvec3<T>& V0, const glm::tvec3<T>& V1, const glm::tvec3<T>& V2,
                                     const glm::tvec3<T>& U0, const glm::tvec3<T>& U1, const glm::tvec3<T>& U2,
                                     bool& coplanar, glm::tvec3<T>& isectpt1, glm::tvec3<T>& isectpt2,
                                     bool check_isect_endpoints, double& debug_reduced_to_0__) {
        glm::tvec3<T> E1 = {0.0, 0.0, 0.0};
        glm::tvec3<T> E2 = {0.0, 0.0, 0.0};
        glm::tvec3<T> N1 = {0.0, 0.0, 0.0};
        glm::tvec3<T> N2 = {0.0, 0.0, 0.0};
        T d1, d2;
        T du0, du1, du2, dv0, dv1, dv2;
        glm::tvec3<T> D = {0.0, 0.0, 0.0};
        T isect1[2], isect2[2];
        glm::tvec3<T> isectpointA1 = {0.0, 0.0, 0.0};
        glm::tvec3<T> isectpointA2 = {0.0, 0.0, 0.0};
        glm::tvec3<T> isectpointB1 = {0.0, 0.0, 0.0};
        glm::tvec3<T> isectpointB2 = {0.0, 0.0, 0.0};
        T du0du1, du0du2, dv0dv1, dv0dv2;
        size_t index;
        T vp0, vp1, vp2;
        T up0, up1, up2;
        T b, c, max;
        size_t smallest1, smallest2;

        /* compute plane equation of triangle(V0,V1,V2) */
        sub(E1, V1, V0);    // E1 = V1 - V0
        sub(E2, V2, V0);    // E2 = V2 - V0
        cross(N1, E1, E2);  // N1 = E1 \times E2
        normalize(N1);      // normalization; added by ZK
        d1 = -dot(N1, V0);  // d1 = -N1.V0
        /* plane equation 1: N1.X + d1 = 0 */

        /* put U0, U1, U2 into plane equation 1 to compute signed distances to the plane */
        du0 = dot(N1, U0) + d1;
        du1 = dot(N1, U1) + d1;
        du2 = dot(N1, U2) + d1;

        auto conditionally_round_to_zero = [&debug_reduced_to_0__](T& d) -> void {
            double x = fabs(d);
            if (x >= EPSILON || x == 0.0)  //
                return;                    // RETURN
            if (x > debug_reduced_to_0__) debug_reduced_to_0__ = x;
            d = 0.0;
        };

        // In the instructions below, du0, du1 and du2 may be conditionally/artifically set to 0
        // This does not seem to influence any arithmetic computations (vertex coordinates are left intact)
        // However, their value equal to zero is used further below in conditional statements
        //   to indicate that a vertex from a face is coplanar with the other face
        //   and this bit of information is used as a branch selector in the algorithmic tree
        // Thus, the value of EPSILON controls the definition of 4 vertices being considered "coplanar"
        debug_reduced_to_0__ = 0.0;        // to be removed once the code has been debugged && understood
        conditionally_round_to_zero(du0);  // if (fabs(du0) < EPSILON) du0 = 0.0;
        conditionally_round_to_zero(du1);  // if (fabs(du1) < EPSILON) du1 = 0.0;
        conditionally_round_to_zero(du2);  // if (fabs(du2) < EPSILON) du2 = 0.0;

        du0du1 = du0 * du1;
        du0du2 = du0 * du2;

        if (du0du1 > 0.0f && du0du2 > 0.0f) /* same sign on all of them + not equal 0 ? */
            return false;                   /* RETURN: no intersection occurs */

        /* compute plane of triangle (U0,U1,U2) */
        sub(E1, U1, U0);    // E1 = U1 - U0
        sub(E2, U2, U0);    // E2 = U2 - U0
        cross(N2, E1, E2);  // N2 = E1 \times E2
        normalize(N2);      // normalization; added by ZK
        d2 = -dot(N2, U0);  // d2 = -N2.U0
        /* plane equation 2: N2.X+d2=0 */

        /* put V0,V1,V2 into plane equation 2 */
        dv0 = dot(N2, V0) + d2;
        dv1 = dot(N2, V1) + d2;
        dv2 = dot(N2, V2) + d2;

        conditionally_round_to_zero(dv0);  // if (fabs(dv0) < EPSILON) dv0 = 0.0;
        conditionally_round_to_zero(dv1);  // if (fabs(dv1) < EPSILON) dv1 = 0.0;
        conditionally_round_to_zero(dv2);  // if (fabs(dv2) < EPSILON) dv2 = 0.0;

        dv0dv1 = dv0 * dv1;
        dv0dv2 = dv0 * dv2;

        if (dv0dv1 > 0.0f && dv0dv2 > 0.0f) /* same sign on all of them + not equal 0 ? */
            return false;                   /* RETURN: no intersection occurs */

        /* compute direction of intersection line */
        cross(D, N1, N2);  // D = N1 \times N2 is orthogonal both to N1 and N2, unless both triangles are coplanar

        /*
         * Warning! At this point we do not know if the faces are coplanar or not
         * If they are, then vector D == (0,0,0), so it points at no direction
         * It should be verified whether the code below, especially projection on a plane, is always robust to this
         *  ambiguity
         */

        /* compute the index into the largest component of D */
        max = fabs(D[0]);
        index = 0;
        b = fabs(D[1]);
        c = fabs(D[2]);
        if (b > max) {
            max = b;
            index = 1;
        }
        if (c > max) {
            max = c;
            index = 2;
        }

        /* Projection onto the axis corresponding to index */
        /* This corresponds to projection onto x, y, or z, whichever is "closer" to the direction of isectline D */
        vp0 = V0[index];
        vp1 = V1[index];
        vp2 = V2[index];

        up0 = U0[index];
        up1 = U1[index];
        up2 = U2[index];

        /* compute interval for triangle 1 */
        coplanar = compute_intervals_isectline(V0, V1, V2, vp0, vp1, vp2, dv0, dv1, dv2, dv0dv1, dv0dv2, isect1[0],
                                               isect1[1], isectpointA1, isectpointA2);

        /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
        /* !!!!!!!  at this point we know whether the triangles are coplanar  !!!!!!!! */
        /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

        if (coplanar) {
            return coplanar_tri_tri(N1, V0, V1, V2, U0, U1, U2);  // RETURN
        }

        /* compute interval for triangle 2 */
        compute_intervals_isectline(U0, U1, U2, up0, up1, up2, du0, du1, du2, du0du1, du0du2, isect2[0], isect2[1],
                                    isectpointB1, isectpointB2);

        smallest1 = sort2(isect1[0], isect1[1]);
        smallest2 = sort2(isect2[0], isect2[1]);

        if (isect1[1] < isect2[0] || isect2[1] < isect1[0])  //
            return false;                                    // RETURN

        /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
        /* !!!!!!!  at this point we know that the triangles intersect  !!!!!!!!! */
        /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

        if (!check_isect_endpoints) {
            return true;  // RETURN
        }

        if (isect2[0] < isect1[0]) {
            if (smallest1 == 0) {
                set(isectpt1, isectpointA1);
            } else {
                set(isectpt1, isectpointA2);
            }

            if (isect2[1] < isect1[1]) {
                if (smallest2 == 0) {
                    set(isectpt2, isectpointB2);
                } else {
                    set(isectpt2, isectpointB1);
                }
            } else {
                if (smallest1 == 0) {
                    set(isectpt2, isectpointA2);
                } else {
                    set(isectpt2, isectpointA1);
                }
            }
        } else {
            if (smallest2 == 0) {
                set(isectpt1, isectpointB1);
            } else {
                set(isectpt1, isectpointB2);
            }

            if (isect2[1] > isect1[1]) {
                if (smallest1 == 0) {
                    set(isectpt2, isectpointA2);
                } else {
                    set(isectpt2, isectpointA1);
                }
            } else {
                if (smallest2 == 0) {
                    set(isectpt2, isectpointB2);
                } else {
                    set(isectpt2, isectpointB1);
                }
            }
        }
        return true;
    }

    // N is a vector orthogonal to the plane defined by V0, V1, and V2.
    //
    static bool coplanar_tri_tri(const glm::tvec3<T>& N, const glm::tvec3<T>& V0, const glm::tvec3<T>& V1,
                                 const glm::tvec3<T>& V2, const glm::tvec3<T>& U0, const glm::tvec3<T>& U1,
                                 const glm::tvec3<T>& U2) {
        glm::tvec3<T> A = {0.0, 0.0, 0.0};
        size_t i0, i1;
        /* first project onto an axis-aligned plane, that maximizes the area */
        /* of the triangles, compute indices: i0, i1. */
        A[X] = fabs(N[X]);
        A[Y] = fabs(N[Y]);
        A[Z] = fabs(N[Z]);
        if (A[X] > A[Y]) {
            if (A[X] > A[Z]) {
                i0 = 1; /* A[X] is greatest, so exclude X==0 from i0, i1 */
                i1 = 2;
            } else {
                i0 = 0; /* A[Z] is greatest, so exclude Z==2 from i0, i1 */
                i1 = 1;
            }
        } else /* A[X]<=A[Y] */
        {
            if (A[Z] > A[Y]) {
                i0 = 0; /* A[Z] is greatest, so exclude Z==2 from i0, i1 */
                i1 = 1;
            } else {
                i0 = 0; /* A[Y] is greatest, so exclude Y==1 from i0, i1  */
                i1 = 2;
            }
        }

        /* test all edges of triangle 1 against the edges of triangle 2 */
        if (edge_against_tri_edge(V0, V1, U0, U1, U2, i0, i1)) {
            return true;
        }
        if (edge_against_tri_edge(V1, V2, U0, U1, U2, i0, i1)) {
            return true;
        }
        if (edge_against_tri_edge(V2, V0, U0, U1, U2, i0, i1)) {
            return true;
        }

        /* finally, test if tri1 is totally contained in tri2 or vice versa */
        if (point_in_tri(V0, U0, U1, U2, i0, i1)) {
            return true;
        }
        if (point_in_tri(U0, V0, V1, V2, i0, i1)) {
            return true;
        }

        return false;
    }

private:
    inline static bool compute_intervals_isectline(const glm::tvec3<T>& VERT0, const glm::tvec3<T>& VERT1,
                                                   const glm::tvec3<T>& VERT2, const T VV0, const T VV1,
                                                   const T VV2, const T D0, const T D1,
                                                   const T D2, const T D0D1, const T D0D2,
                                                   T& isect0, T& isect1, glm::tvec3<T>& isectpoint0,
                                                   glm::tvec3<T>& isectpoint1) {
        if (D0D1 > 0.0) {
            /* here we know that D0D2<=0.0 */
            /* that is D0, D1 are on the same side, D2 on the other or on the plane */
            isect2(VERT2, VERT0, VERT1, VV2, VV0, VV1, D2, D0, D1, isect0, isect1, isectpoint0, isectpoint1);
        } else if (D0D2 > 0.0f) {
            /* here we know that d0d1<=0.0 */
            isect2(VERT1, VERT0, VERT2, VV1, VV0, VV2, D1, D0, D2, isect0, isect1, isectpoint0, isectpoint1);
        } else if (D1 * D2 > 0.0f || D0 != 0.0f) {
            /* here we know that d0d1<=0.0 or that D0!=0.0 */
            isect2(VERT0, VERT1, VERT2, VV0, VV1, VV2, D0, D1, D2, isect0, isect1, isectpoint0, isectpoint1);
        } else if (D1 != 0.0f) {
            isect2(VERT1, VERT0, VERT2, VV1, VV0, VV2, D1, D0, D2, isect0, isect1, isectpoint0, isectpoint1);
        } else if (D2 != 0.0f) {
            isect2(VERT2, VERT0, VERT1, VV2, VV0, VV1, D2, D0, D1, isect0, isect1, isectpoint0, isectpoint1);
        } else {
            /* triangles are coplanar */
            return true;
        }
        return false;
    }
};

}
#endif  // TRIANGLEINTERSECTS_H
