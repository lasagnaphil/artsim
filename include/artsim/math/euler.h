//
// Created by lasagnaphil on 19. 10. 1..
//

#ifndef GENGINE_GLMX_EULER_H
#define GENGINE_GLMX_EULER_H

/*** Order type constants, constructors, extractors ***/
/* There are 24 possible conventions, designated by:    */
/*	  o EulAxI = axis used initially		    */
/*	  o EulPar = parity of axis permutation		    */
/*	  o EulRep = repetition of initial axis as last	    */
/*	  o EulFrm = frame from which axes are taken	    */
/* Axes I,J,K will be a permutation of X,Y,Z.	    */
/* Axis H will be either I or K, depending on EulRep.   */
/* Frame S takes axes from initial static frame.	    */
/* If ord = (AxI=X, Par=Even, Rep=No, Frm=S), then	    */
/* {a,b,c,ord} means Rz(c)Ry(b)Rx(a), where Rz(c)v	    */
/* rotates v around Z by c radians.			    */
#define EulFrmS	     0
#define EulFrmR	     1
#define EulFrm(ord)  ((unsigned)(ord)&1)
#define EulRepNo     0
#define EulRepYes    1
#define EulRep(ord)  (((unsigned)(ord)>>1)&1)
#define EulParEven   0
#define EulParOdd    1
#define EulPar(ord)  (((unsigned)(ord)>>2)&1)
/* this code is merely a quick (and legal!) way to set arrays, EulSafe being 0,1,2,0 */
#define EulSafe	     "\000\001\002\000"
#define EulNext	     "\001\002\000\001"
#define EulAxI(ord)  ((int)(EulSafe[(((unsigned)(ord)>>3)&3)]))
#define EulAxJ(ord)  ((int)(EulNext[EulAxI(ord)+(EulPar(ord)==EulParOdd)]))
#define EulAxK(ord)  ((int)(EulNext[EulAxI(ord)+(EulPar(ord)!=EulParOdd)]))
#define EulAxH(ord)  ((EulRep(ord)==EulRepNo)?EulAxK(ord):EulAxI(ord))
/* EulGetOrd unpacks all useful information about order simultaneously. */
#define EulGetOrd(ord,i,j,k,h,n,s,f) {unsigned o=(unsigned)ord;f=o&1;o>>=1;s=o&1;o>>=1;\
    n=o&1;o>>=1;i=EulSafe[o&3];j=EulNext[i+n];k=EulNext[i+1-n];h=s?k:i;}
/* EulOrd creates an order value between 0 and 23 from 4-tuple choices. */
#define EulOrd(i,p,r,f)	   (((((((i)<<1)+(p))<<1)+(r))<<1)+(f))
/* Static axes */
#define EulOrdXYZs    EulOrd(0,EulParEven,EulRepNo,EulFrmS)
#define EulOrdXYXs    EulOrd(0,EulParEven,EulRepYes,EulFrmS)
#define EulOrdXZYs    EulOrd(0,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdXZXs    EulOrd(0,EulParOdd,EulRepYes,EulFrmS)
#define EulOrdYZXs    EulOrd(1,EulParEven,EulRepNo,EulFrmS)
#define EulOrdYZYs    EulOrd(1,EulParEven,EulRepYes,EulFrmS)
#define EulOrdYXZs    EulOrd(1,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdYXYs    EulOrd(1,EulParOdd,EulRepYes,EulFrmS)
#define EulOrdZXYs    EulOrd(2,EulParEven,EulRepNo,EulFrmS)
#define EulOrdZXZs    EulOrd(2,EulParEven,EulRepYes,EulFrmS)
#define EulOrdZYXs    EulOrd(2,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdZYZs    EulOrd(2,EulParOdd,EulRepYes,EulFrmS)
/* Rotating axes */
#define EulOrdZYXr    EulOrd(0,EulParEven,EulRepNo,EulFrmR)
#define EulOrdXYXr    EulOrd(0,EulParEven,EulRepYes,EulFrmR)
#define EulOrdYZXr    EulOrd(0,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdXZXr    EulOrd(0,EulParOdd,EulRepYes,EulFrmR)
#define EulOrdXZYr    EulOrd(1,EulParEven,EulRepNo,EulFrmR)
#define EulOrdYZYr    EulOrd(1,EulParEven,EulRepYes,EulFrmR)
#define EulOrdZXYr    EulOrd(1,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdYXYr    EulOrd(1,EulParOdd,EulRepYes,EulFrmR)
#define EulOrdYXZr    EulOrd(2,EulParEven,EulRepNo,EulFrmR)
#define EulOrdZXZr    EulOrd(2,EulParEven,EulRepYes,EulFrmR)
#define EulOrdXYZr    EulOrd(2,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdZYZr    EulOrd(2,EulParOdd,EulRepYes,EulFrmR)

namespace glmx {

template <class T>
inline glm::tquat<T> eulerToQuat(glm::tvec3<T> ea, uint32_t ord=EulOrdXYZs)
{
    glm::tquat<T> qu;
    T a[3], ti, tj, th, ci, cj, ch, si, sj, sh, cc, cs, sc, ss;
    int i,j,k,h,n,s,f;
    EulGetOrd(ord,i,j,k,h,n,s,f);
    if (f==EulFrmR) {T t = ea.x; ea.x = ea.z; ea.z = t;}
    if (n==EulParOdd) ea.y = -ea.y;
    ti = ea.x*0.5f; tj = ea.y*0.5f; th = ea.z*0.5f;
    ci = cosf(ti);  cj = cosf(tj);  ch = cosf(th);
    si = sinf(ti);  sj = sinf(tj);  sh = sinf(th);
    cc = ci*ch; cs = ci*sh; sc = si*ch; ss = si*sh;
    if (s==EulRepYes) {
        a[i] = cj*(cs + sc);	/* Could speed up with */
        a[j] = sj*(cc + ss);	/* trig identities. */
        a[k] = sj*(cs - sc);
        qu.w = cj*(cc - ss);
    } else {
        a[i] = cj*sc - sj*cs;
        a[j] = cj*ss + sj*cc;
        a[k] = cj*cs - sj*sc;
        qu.w = cj*cc + sj*ss;
    }
    if (n==EulParOdd) a[j] = -a[j];
    qu.x = a[0]; qu.y = a[1]; qu.z = a[2];
    return qu;
}

/* Construct matrix from Euler angles (in radians). */
template <class T>
inline glm::tmat4x4<T> eulerToMat(glm::tvec3<T> ea, uint32_t ord=EulOrdXYZs)
{
    glm::tmat4x4<T> M;
    T ti, tj, th, ci, cj, ch, si, sj, sh, cc, cs, sc, ss;
    int i,j,k,h,n,s,f;
    EulGetOrd(ord,i,j,k,h,n,s,f);
    if (f==EulFrmR) {T t = ea.x; ea.x = ea.z; ea.z = t;}
    if (n==EulParOdd) {ea.x = -ea.x; ea.y = -ea.y; ea.z = -ea.z;}
    ti = ea.x;	  tj = ea.y;	th = ea.z;
    ci = glm::cos(ti); cj = glm::cos(tj); ch = glm::cos(th);
    si = glm::sin(ti); sj = glm::sin(tj); sh = glm::sin(th);
    cc = ci*ch; cs = ci*sh; sc = si*ch; ss = si*sh;
    if (s==EulRepYes) {
        M[i][i] = cj;	  M[i][j] =  sj*si;    M[i][k] =  sj*ci;
        M[j][i] = sj*sh;  M[j][j] = -cj*ss+cc; M[j][k] = -cj*cs-sc;
        M[k][i] = -sj*ch; M[k][j] =  cj*sc+cs; M[k][k] =  cj*cc-ss;
    } else {
        M[i][i] = cj*ch; M[i][j] = sj*sc-cs; M[i][k] = sj*cc+ss;
        M[j][i] = cj*sh; M[j][j] = sj*ss+cc; M[j][k] = sj*cs-sc;
        M[k][i] = -sj;	 M[k][j] = cj*si;    M[k][k] = cj*ci;
    }
    M[3][0]=M[3][1]=M[3][2]=M[0][3]=M[1][3]=M[2][3]=0.f; M[3][3]=1.f;
    return M;
}

/* Convert matrix to Euler angles (in radians). */
template <class T>
inline glm::tvec3<T> matToEuler(const glm::tmat4x4<T>& M, int order)
{
    glm::tvec3<T> ea;
    int i,j,k,h,n,s,f;
    EulGetOrd(order,i,j,k,h,n,s,f);
    if (s==EulRepYes) {
        T sy = glm::sqrt(M[i][j]*M[i][j] + M[i][k]*M[i][k]);
        if (sy > 16*FLT_EPSILON) {
            ea.x = glm::atan(M[i][j], M[i][k]);
            ea.y = glm::atan(sy, M[i][i]);
            ea.z = glm::atan(M[j][i], -M[k][i]);
        } else {
            ea.x = glm::atan(-M[j][k], M[j][j]);
            ea.y = glm::atan(sy, M[i][i]);
            ea.z = 0;
        }
    } else {
        T cy = glm::sqrt(M[i][i]*M[i][i] + M[j][i]*M[j][i]);
        if (cy > 16*FLT_EPSILON) {
            ea.x = glm::atan(M[k][j], M[k][k]);
            ea.y = glm::atan(-M[k][i], cy);
            ea.z = glm::atan(M[j][i], M[i][i]);
        } else {
            ea.x = glm::atan(-M[j][k], M[j][j]);
            ea.y = glm::atan(-M[k][i], cy);
            ea.z = 0;
        }
    }
    if (n==EulParOdd) {ea.x = -ea.x; ea.y = - ea.y; ea.z = -ea.z;}
    if (f==EulFrmR) {float t = ea.x; ea.x = ea.z; ea.z = t;}
    return ea;
}

template <class T>
inline glm::tvec3<T> quatToEuler(glm::tquat<T> q, int order)
{
    glm::tmat4x4<T> M;
    T Nq = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    T s = (Nq > 0.f) ? (2.f / Nq) : 0.f;
    T xs = q.x*s,	  ys = q.y*s,	 zs = q.z*s;
    T wx = q.w*xs,	  wy = q.w*ys,	 wz = q.w*zs;
    T xx = q.x*xs,	  xy = q.x*ys,	 xz = q.x*zs;
    T yy = q.y*ys,	  yz = q.y*zs,	 zz = q.z*zs;
    M[0][0] = 1.f - (yy + zz); M[0][1] = xy - wz; M[0][2] = xz + wy;
    M[1][0] = xy + wz; M[1][1] = 1.f - (xx + zz); M[1][2] = yz - wx;
    M[2][0] = xz - wy; M[2][1] = yz + wx; M[2][2] = 1.f - (xx + yy);
    M[3][0]=M[3][1]=M[3][2]=M[0][3]=M[1][3]=M[2][3]=0.f; M[3][3]=1.f;
    return matToEuler(M, order);
}

}

#undef X
#undef Y
#undef Z
#undef W


#endif //GENGINE_EULER_H
