//
// Created by lasagnaphil on 21. 2. 23..
//

#ifndef ARTSIM_DIST_H
#define ARTSIM_DIST_H

// Modified from Geometric Tools code

// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2021
// Distributed under the Boost Software License, Version 1.0.
// https://www.boost.org/LICENSE_1_0.txt
// https://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// Version: 4.0.2019.08.13

namespace glmx {

template <class T>
struct PointTriangleDistResult
{
    T sqrDistance;
    // barycentric coordinates for triangle.v[3]
    T parameter[3];
    glm::tvec3<T> closest;
};

template <typename T>
PointTriangleDistResult<T> point_triangle_dist(
        glm::tvec3<T> const& point, glm::tvec3<T> tri1, glm::tvec3<T> tri2, glm::tvec3<T> tri3)
{
    glm::tvec3<T> diff = point - tri1;
    glm::tvec3<T> edge0 = tri2 - tri1;
    glm::tvec3<T> edge1 = tri3 - tri1;
    T a00 = glm::dot(edge0, edge0);
    T a01 = glm::dot(edge0, edge1);
    T a11 = glm::dot(edge1, edge1);
    T b0 = -glm::dot(diff, edge0);
    T b1 = -glm::dot(diff, edge1);
    T const zero = (T)0;
    T const one = (T)1;
    T det = a00 * a11 - a01 * a01;
    T t0 = a01 * b1 - a11 * b0;
    T t1 = a01 * b0 - a00 * b1;

    if (t0 + t1 <= det)
    {
        if (t0 < zero)
        {
            if (t1 < zero)  // region 4
            {
                if (b0 < zero)
                {
                    t1 = zero;
                    if (-b0 >= a00)  // V1
                    {
                        t0 = one;
                    }
                    else  // E01
                    {
                        t0 = -b0 / a00;
                    }
                }
                else
                {
                    t0 = zero;
                    if (b1 >= zero)  // V0
                    {
                        t1 = zero;
                    }
                    else if (-b1 >= a11)  // V2
                    {
                        t1 = one;
                    }
                    else  // E20
                    {
                        t1 = -b1 / a11;
                    }
                }
            }
            else  // region 3
            {
                t0 = zero;
                if (b1 >= zero)  // V0
                {
                    t1 = zero;
                }
                else if (-b1 >= a11)  // V2
                {
                    t1 = one;
                }
                else  // E20
                {
                    t1 = -b1 / a11;
                }
            }
        }
        else if (t1 < zero)  // region 5
        {
            t1 = zero;
            if (b0 >= zero)  // V0
            {
                t0 = zero;
            }
            else if (-b0 >= a00)  // V1
            {
                t0 = one;
            }
            else  // E01
            {
                t0 = -b0 / a00;
            }
        }
        else  // region 0, interior
        {
            T invDet = one / det;
            t0 *= invDet;
            t1 *= invDet;
        }
    }
    else
    {
        T tmp0, tmp1, numer, denom;

        if (t0 < zero)  // region 2
        {
            tmp0 = a01 + b0;
            tmp1 = a11 + b1;
            if (tmp1 > tmp0)
            {
                numer = tmp1 - tmp0;
                denom = a00 - (T)2 * a01 + a11;
                if (numer >= denom)  // V1
                {
                    t0 = one;
                    t1 = zero;
                }
                else  // E12
                {
                    t0 = numer / denom;
                    t1 = one - t0;
                }
            }
            else
            {
                t0 = zero;
                if (tmp1 <= zero)  // V2
                {
                    t1 = one;
                }
                else if (b1 >= zero)  // V0
                {
                    t1 = zero;
                }
                else  // E20
                {
                    t1 = -b1 / a11;
                }
            }
        }
        else if (t1 < zero)  // region 6
        {
            tmp0 = a01 + b1;
            tmp1 = a00 + b0;
            if (tmp1 > tmp0)
            {
                numer = tmp1 - tmp0;
                denom = a00 - (T)2 * a01 + a11;
                if (numer >= denom)  // V2
                {
                    t1 = one;
                    t0 = zero;
                }
                else  // E12
                {
                    t1 = numer / denom;
                    t0 = one - t1;
                }
            }
            else
            {
                t1 = zero;
                if (tmp1 <= zero)  // V1
                {
                    t0 = one;
                }
                else if (b0 >= zero)  // V0
                {
                    t0 = zero;
                }
                else  // E01
                {
                    t0 = -b0 / a00;
                }
            }
        }
        else  // region 1
        {
            numer = a11 + b1 - a01 - b0;
            if (numer <= zero)  // V2
            {
                t0 = zero;
                t1 = one;
            }
            else
            {
                denom = a00 - (T)2 * a01 + a11;
                if (numer >= denom)  // V1
                {
                    t0 = one;
                    t1 = zero;
                }
                else  // 12
                {
                    t0 = numer / denom;
                    t1 = one - t0;
                }
            }
        }
    }

    PointTriangleDistResult<T> result;
    result.parameter[0] = one - t0 - t1;
    result.parameter[1] = t0;
    result.parameter[2] = t1;
    result.closest = tri1 + t0 * edge0 + t1 * edge1;
    diff = point - result.closest;
    result.sqrDistance = glm::dot(diff, diff);
    return result;
}

}

#endif //ARTSIM_DIST_H
