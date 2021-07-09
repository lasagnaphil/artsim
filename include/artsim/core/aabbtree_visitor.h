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
    const glm::tvec3<T> *verts;
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
    if( aabb.isEmpty() ){
        printf("PointInTet Error: Empty AABB\n");
        exit(EXIT_FAILURE);
    }
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
    T left_ed = left.squaredExteriorDistance( point );
    return left_ed <= right.squaredExteriorDistance( point );
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

/*
template <class T>
struct RayCast : public AABBTreeVisitor<T> {
    using AABB = glmx::tbox<3, T>;

    glm::tvec3<T> pos;
    glm::tvec3<T> dir;

    T hit_t;
    int hit_tri;

    RayCast(glm::tvec3<T> pos, glm::tvec3<T> dir) : pos(pos), dir(dir) {}
    bool hit_aabb(const AABB &aabb);
    bool hit_prim(int prim);
    bool check_left_first(const AABB &left, const AABB &right);
};

template <class T>
bool RayCast<T>::hit_aabb(const RayCast::AABB &aabb) {
    return false;
}

template <class T>
bool RayCast<T>::hit_prim(int prim) {
    return false;
}

template <class T>
bool RayCast<T>::check_left_first(const RayCast::AABB &left, const RayCast::AABB &right) {
    return false;
}
 */

#endif //EOS_SCAN_TO_HUMAN_AABBTREE_VISITOR_H
