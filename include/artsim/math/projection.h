//
// Created by lasagnaphil on 21. 6. 28..
//

#ifndef EOS_SCAN_TO_HUMAN_PROJECTION_H
#define EOS_SCAN_TO_HUMAN_PROJECTION_H

namespace glmx {

template <class T>
glm::tvec3<T> point_on_triangle( const glm::tvec3<T> &point, const glm::tvec3<T> &p1, const glm::tvec3<T> &p2, const glm::tvec3<T> &p3 ){

    glm::tvec3<T> edge0 = p2 - p1;
    glm::tvec3<T> edge1 = p3 - p1;
    glm::tvec3<T> v0 = p1 - point;

    T a = edge0.dot( edge0 );
    T b = edge0.dot( edge1 );
    T c = edge1.dot( edge1 );
    T d = edge0.dot( v0 );
    T e = edge1.dot( v0 );
    T det = a*c - b*b;
    T s = b*e - c*d;
    T t = b*d - a*e;

    const T zero(0);
    const T one(1);

    if ( s + t < det ) {
        if ( s < zero ) {
            if ( t < zero ) {
                if ( d < zero ) {
                    s = myclamp( -d/a );
                    t = zero;
                }
                else {
                    s = zero;
                    t = myclamp( -e/c );
                }
            }
            else {
                s = zero;
                t = myclamp( -e/c );
            }
        }
        else if ( t < zero ) {
            s = myclamp( -d/a );
            t = zero;
        }
        else {
            T invDet = one / det;
            s *= invDet;
            t *= invDet;
        }
    }
    else {
        if ( s < zero ) {
            T tmp0 = b+d;
            T tmp1 = c+e;
            if ( tmp1 > tmp0 ) {
                T numer = tmp1 - tmp0;
                T denom = a-T(2)*b+c;
                s = myclamp( numer/denom );
                t = one-s;
            }
            else {
                t = myclamp( -e/c );
                s = zero;
            }
        }
        else if ( t < zero ) {
            if ( a+d > b+e ) {
                T numer = c+e-b-d;
                T denom = a-T(2)*b+c;
                s = myclamp( numer/denom );
                t = one-s;
            }
            else {
                s = myclamp( -e/c );
                t = zero;
            }
        }
        else {
            T numer = c+e-b-d;
            T denom = a-T(2)*b+c;
            s = myclamp( numer/denom );
            t = one - s;
        }
    }

    return ( p1 + edge0*s + edge1*t );

} // end project triangle


template <class T>
glm::tvec3<T> point_on_sphere( const glm::tvec3<T> &point, const glm::tvec3<T> &center, const T &rad ){
    glm::tvec3<T> dir = point-center;
    dir.normalize();
    return ( center + dir*rad );
} // end project sphere


template <class T>
glm::tvec3<T> point_on_box( const glm::tvec3<T> &point, const glm::tvec3<T> &bmin, const glm::tvec3<T> &bmax ){
    // Loops through axes and moves point to nearest surface
    glm::tvec3<T> x = point;
    T dx = std::numeric_limits<T>::max();
    for( int i=0; i<3; ++i ){
        T dx_max = std::abs(bmax[i]-point[i]);
        T dx_min = std::abs(bmin[i]-point[i]);
        if( dx_max < dx ){
            x = point;
            x[i] = bmax[i];
            dx = dx_max;
        }
        if( dx_min < dx ){
            x = point;
            x[i] = bmin[i];
            dx = dx_min;
        }
    }
    return x;
} // end project box


template <class T>
bool check_norm( const glm::tvec3<T> &point,
                 const glm::tvec3<T> &p0, const glm::tvec3<T> &p1, const glm::tvec3<T> &p2, const glm::tvec3<T> &p3 ){
    const glm::tvec3<T> n = (p1 - p0).cross(p2 - p0);
    const T dp3 = n.dot(p3 - p0);
    const T dp = n.dot(point - p0);
    return (dp3*dp>0);
}


template <class T>
bool point_in_tet( const glm::tvec3<T> &point,
                               const glm::tvec3<T> &p0, const glm::tvec3<T> &p1, const glm::tvec3<T> &p2, const glm::tvec3<T> &p3 ){
    return check_norm<T>(point, p0, p1, p2, p3) && check_norm<T>(point, p1, p2, p3, p0) &&
           check_norm<T>(point, p2, p3, p0, p1) && check_norm<T>(point, p3, p0, p1, p2);
}

}

#endif //EOS_SCAN_TO_HUMAN_PROJECTION_H
