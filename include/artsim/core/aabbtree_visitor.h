//
// Created by lasagnaphil on 21. 6. 28..
//

#ifndef EOS_SCAN_TO_HUMAN_AABBTREE_VISITOR_H
#define EOS_SCAN_TO_HUMAN_AABBTREE_VISITOR_H

#include <vector>
#include <cstdio>
#include <artsim/math/box.h>
#include <artsim/math/projection.h>

template <class T>
struct AABBTreeVisitor {
    using AABB = glmx::tbox<3, T>;

    virtual bool hit_aabb(const AABB& aabb) = 0;
    virtual bool hit_prim(int prim_id) = 0;
    virtual bool check_left_first(const AABB& left, const AABB& right) = 0;
};

template <class T>
struct AABBTreePairVisitor {
    using AABB = glmx::tbox<3, T>;

    virtual bool hit_prim_pair(int prim1_id, int prim2_id) = 0;
};

// Point in tet
template <class T>
struct PointInTet : public AABBTreeVisitor<T> {
    using AABB = glmx::tbox<3, T>;

    glm::tvec3<T> point; // query point
    int hit_tet; // intersected tet
    std::vector<int> skip_vert_idx;  // vert index to skip (for self collision)
    const T *verts;
    const int *inds;
    PointInTet( glm::tvec3<T> point_, const T *verts_, const int *inds_ );
    bool hit_aabb( const AABB &aabb );
    bool hit_prim( int prim );
    bool check_left_first( const AABB &left, const AABB &right );
};

template <class T>
PointInTet<T>::PointInTet( glm::tvec3<T> point_, const T *verts_, const int *inds_ ) :
        point(point_), verts(verts_), inds(inds_), hit_tet(-1) {}

template <class T>
bool PointInTet<T>::hit_aabb( const AABB &aabb ){
    return aabb.contains(point);
}

template <class T>
bool PointInTet<T>::hit_prim( int prim ){
    glm::ivec4 tet( inds[prim*4+0], inds[prim*4+1], inds[prim*4+2], inds[prim*4+3] );
    int n_skip = skip_vert_idx.size();
    for( int i=0; i<n_skip; ++i ){
        for( int j=0; j<4; ++j ){
            if( skip_vert_idx[i]==tet[j] ){ return false; }
        }
    }
    glm::tvec3<T> v0( verts[tet[0]*3+0], verts[tet[0]*3+1], verts[tet[0]*3+2] );
    glm::tvec3<T> v1( verts[tet[1]*3+0], verts[tet[1]*3+1], verts[tet[1]*3+2] );
    glm::tvec3<T> v2( verts[tet[2]*3+0], verts[tet[2]*3+1], verts[tet[2]*3+2] );
    glm::tvec3<T> v3( verts[tet[3]*3+0], verts[tet[3]*3+1], verts[tet[3]*3+2] );
    if( glmx::point_in_tet<T>( point, v0, v1, v2, v3 ) ){
        hit_tet = prim;
        return true;
    }
    return false;
}

template <class T>
bool PointInTet<T>::check_left_first( const AABB &left, const AABB &right ){
    return glmx::distance2(left, point) <= glmx::distance2(right, point);
}

// Nearest point on surface
template <class T>
struct NearestTriangle : public AABBTreeVisitor<T> {
    using AABB = glmx::tbox<3, T>;

    glm::tvec3<T> point; // query point
    glm::tvec3<T> proj; // nearest point on hit_tri
    int hit_tri; // triangle idx
    std::vector<int> skip_vert_idx; // vert index to skip (for self collision)
    T curr_nearest; // current nearest distance to tri
    const T *verts;
    const int *inds;
    NearestTriangle( glm::tvec3<T> point_, const T *verts_, const int *inds_ );
    bool hit_aabb( const AABB &aabb );
    bool hit_prim( int prim );
    bool check_left_first( const AABB &left, const AABB &right );
};

template <class T>
NearestTriangle<T>::NearestTriangle( glm::tvec3<T> point_, const T *verts_, const int *inds_ ) :
        point(point_), verts(verts_), inds(inds_), hit_tri(-1), proj(-1,-1,-1),
        curr_nearest(std::numeric_limits<T>::max()) {}

template <class T>
bool NearestTriangle<T>::hit_aabb( const AABB &aabb ){
    return glmx::distance(aabb, point) < curr_nearest;
}

template <class T>
bool NearestTriangle<T>::hit_prim( int prim ){
    glm::ivec3 tri( inds[prim*3+0], inds[prim*3+1], inds[prim*3+2] );
    int n_skip = skip_vert_idx.size();
    for( int i=0; i<n_skip; ++i ){
        for( int j=0; j<3; ++j ){
            if( skip_vert_idx[i]==tri[j] ){ return false; }
        }
    }
    glm::tvec3<T> v0( verts[tri[0]*3+0], verts[tri[0]*3+1], verts[tri[0]*3+2] );
    glm::tvec3<T> v1( verts[tri[1]*3+0], verts[tri[1]*3+1], verts[tri[1]*3+2] );
    glm::tvec3<T> v2( verts[tri[2]*3+0], verts[tri[2]*3+1], verts[tri[2]*3+2] );

    glm::tvec3<T> p = glmx::point_on_triangle( point, v0, v1, v2 );
    T dist = glm::distance2(p, point);
    if( dist > curr_nearest ){ return false; }

    curr_nearest = dist;
    hit_tri = prim;
    proj = p;
    return false; // return false to keep checking other tris
}

template <class T>
bool NearestTriangle<T>::check_left_first( const AABB &left, const AABB &right ){
    return glmx::distance2(left, point) < glmx::distance2(right, point);
}

template <class T>
struct RayCastToTriMesh : public AABBTreeVisitor<T> {
    using AABB = glmx::tbox<3, T>;

    glm::tvec3<T> pstart;
    glm::tvec3<T> pend;
    glm::tvec3<T> dir;
    glm::tvec3<T> dir_inv;
    T max_dist;

    const glm::tvec3<T>* verts;
    const glm::ivec3* tris;

    glm::tvec3<T> hit_point;
    T hit_t;
    int hit_tri;

    RayCastToTriMesh(glm::tvec3<T> pos, glm::tvec3<T> dir, T max_dist,
                     const glm::tvec3<T>* verts, const glm::ivec3* tris)
        : pstart(pos), dir(dir), max_dist(max_dist),
          verts(verts), tris(tris)
    {
        dir_inv = glm::tvec3<T>(1) / dir;
        pend = pstart + max_dist*dir;
    }

    bool hit_aabb(const AABB &aabb);
    bool hit_prim(int prim);
    bool check_left_first(const AABB &left, const AABB &right);
};

// Slab method for box-ray collision.
// Source: https://tavianator.com/2011/ray_box.html and https://tavianator.com/2015/ray_box_nan.html
template <class T>
bool RayCastToTriMesh<T>::hit_aabb(const RayCastToTriMesh::AABB &aabb) {
    T tx1 = (aabb.lo.x - pstart.x) * dir_inv.x;
    T tx2 = (aabb.hi.x - pstart.x) * dir_inv.x;
    T tmin = glm::min(tx1, tx2);
    T tmax = glm::max(tx1, tx2);
    T ty1 = (aabb.lo.y - pstart.y) * dir_inv.y;
    T ty2 = (aabb.hi.y - pstart.y) * dir_inv.y;
    tmin = glm::max(tmin, glm::min(glm::min(ty1, ty2), std::numeric_limits<T>::infinity()));
    tmax = glm::min(tmax, glm::max(glm::max(ty1, ty2), -std::numeric_limits<T>::infinity()));
    T tz1 = (aabb.lo.z - pstart.z) * dir_inv.z;
    T tz2 = (aabb.hi.z - pstart.z) * dir_inv.z;
    tmin = glm::max(tmin, glm::min(glm::min(tz1, tz2), std::numeric_limits<T>::infinity()));
    tmax = glm::min(tmax, glm::max(glm::max(tz1, tz2), -std::numeric_limits<T>::infinity()));
    return tmax >= tmin;
}

// Moller-Trumbore intersection algorithm.
// Source: https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm

template <class T>
inline T ray_triangle_intersection(const glm::tvec3<T>& ray_from, const glm::tvec3<T>& ray_dir,
                                   const glm::tvec3<T>& vertex0, const glm::tvec3<T>& vertex1, const glm::tvec3<T>& vertex2) {
    const T EPSILON = 1e-7;
    glm::tvec3<T> edge1, edge2, h, s, q;
    T a,f,u,v;
    edge1 = vertex1 - vertex0;
    edge2 = vertex2 - vertex0;
    h = glm::cross(ray_dir, edge2);
    a = glm::dot(edge1, h);
    if (a > -EPSILON && a < EPSILON)
        return false;    // This ray is parallel to this triangle.
    f = T(1)/a;
    s = ray_from - vertex0;
    u = f * glm::dot(s, h);
    if (u < T(0) || u > T(1))
        return false;
    q = glm::cross(s, edge1);
    v = f * glm::dot(ray_dir, q);
    if (v < T(0) || u + v > T(1))
        return false;
    // At this stage we can compute t to find out where the intersection point is on the line.
    return f * glm::dot(edge2, q);
}

template <class T>
bool RayCastToTriMesh<T>::hit_prim(int prim) {
    auto tri = tris[prim];
    auto v0 = verts[tri[0]];
    auto v1 = verts[tri[1]];
    auto v2 = verts[tri[2]];
    hit_tri = prim;
    hit_t = ray_triangle_intersection(pstart, dir, v0, v1, v2);
    hit_point = pstart + hit_t*dir;
    return hit_t >= 0 && hit_t <= max_dist;
}

template <class T>
bool RayCastToTriMesh<T>::check_left_first(const RayCastToTriMesh::AABB &left, const RayCastToTriMesh::AABB &right) {
    return dir.x > 0;
}

#endif //EOS_SCAN_TO_HUMAN_AABBTREE_VISITOR_H
