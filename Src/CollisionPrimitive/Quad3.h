#pragma once

#include "../Maths/Vector3.h"

class Quad3
{
public:
    Vector3 A;
    Vector3 B;
    Vector3 C;
    Vector3 D;
    
    Quad3() {}

    Quad3(const Vector3& iA, const Vector3& iB, const Vector3& iC)
	{
        A = iA;
        B = iB;
        C = iC;
        D = CalculateD(A, B, C);
	}
    
    static Vector3 CalculateD(const Vector3& A, const Vector3& B, const Vector3& C)
    {
        return B + C - A;       // = A + (B - A) + (C - A)
    }
    
    Vector3 GetCenter() const
    {
        return (B + C) * 0.5f;
    }
    
    bool IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
    {
        float tt;
        if (RayIntersectQuad(Origin, Direction, A, B, C, &tt))
        {
            *t = tt;
            return true;
        }
        return false;
    }
    
    static bool RayIntersectQuad(const Vector3& Origin, const Vector3& Direction, const Vector3& A, const Vector3& B, const Vector3& C, float* t)
    {
        float u, v;
        return RayIntersectQuad(Origin, Direction, A, B, C, t, &u, &v);
    }
    
    static bool RayIntersectQuad(const Vector3& Origin, const Vector3& Direction, const Vector3& A, const Vector3& B, const Vector3& C, float* t, float *u, float *v)
    {
        const Vector3 BA = B - A;
        const Vector3 CA = C - A;
        const Vector3 pvec = Direction.Cross(CA);
        const float det = BA.Dot(pvec);

        const float Eps = 0.00001f;
        if (det < Eps)
        {
            return false;
        }

        const Vector3 tvec = Origin - A;

        *u = tvec.Dot(pvec);
        if (*u < 0.0f || *u > det)
        {
            return false;
        }

        const Vector3 qvec = tvec.Cross(BA);

        *v = Direction.Dot(qvec);
        if (*v < 0.0f || *v > det)
        {
            return false;
        }

        *t = CA.Dot(qvec);
        const float invDet = 1.0f / det;
        *t *= invDet;
        *u *= invDet;
        *v *= invDet;
        return true;
    }
    
    static int RayIntersectQuad2(const Vector3& Origin, const Vector3& Direction, const Vector3& A, const Vector3& BA, const Vector3& CA, float* t, float* u, float* v)
    {
        const Vector3 pvec = Direction.Cross(CA);
        const float det = BA.Dot(pvec);

        const float Eps = 0.00001f;
        if(det >- Eps && det < Eps)
        {
            return 0;
        }
        
        const float invDet = 1.0f / det;
        const Vector3 tvec = Origin - A;
        *u = tvec.Dot(pvec) * invDet;
        
        const Vector3 qvec = tvec.Cross(BA);
        *v = Direction.Dot(qvec) * invDet;

        if (*u < 0.0f || *u > 1.0f)
        {
            return 1;
        }
        
        if (*v < 0.0f || *v > 1.0f)
        {
            return 1;
        }

        *t = CA.Dot(qvec) * invDet;
        return 2;
    }
};
