#pragma once

#include <math.h>
#include <stdint.h>
#include <vector>

#undef max
#undef min

#include "ShapeType.h"
#include "../Maths/Matrix3.h"

namespace Geometry
{
	// {x | (x - [X0 + (X1 - X0)*t])^2 <= Radius }, 0 <= t <= 1 }
	class Capsule3
	{
	public:
		Vector3		X0;
		Vector3		X1;
		float		Radius;
		float		Length;

	public:
		Capsule3() {}

		Capsule3(const Vector3& _X0, const Vector3& _X1, float _Radius)
		{
			Init(_X0, _X1, _Radius);
		}

		static constexpr ShapeType	StaticType()
		{
			return ShapeType::CAPSULE;
		}

		inline void		Init(const Vector3& _X0, const Vector3& _X1, float _Radius)
		{
			X0 = _X0;
			X1 = _X1;
			Radius = _Radius;
			Length = (_X1 - _X0).Length();
		}

	public:
		inline Vector3	GetAxis() const
		{
			return X1 - X0;
		}

		inline Vector3	GetUnitAxis() const
		{
			return (X1 - X0).Unit();
		}

		inline Vector3	GetCenter() const
		{
			return (X0 + X1) * 0.5f;
		}

		inline float	GetHeight() const
		{
			return Length;
		}

		inline float	GetHalfHeight() const
		{
			return Length * 0.5f;
		}

		Box3			CalculateBoundingVolume() const
		{
			Box3 box(X0, X0);
			box.Encapsulate(X1);
			box.Thicken(Radius);
			return box;
		}

		bool			IsYAxisAligned() const
		{
			return (X1 - X0).ParallelTo(Vector3::UnitY());
		}

		bool			IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const;

		bool			IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const;
		bool			IntersectSphere(const Vector3& rCenter, float rRadius) const;
		bool 			IntersectSegment(const Vector3& X0, const Vector3& X1) const;
		bool			IntersectCapsule(const Vector3& P0, const Vector3& P1, float rRadius) const;
		bool			IntersectTriangle(const Vector3& A, const Vector3& B, const Vector3& C) const;

		bool			PenetrateSphere(const Vector3& rCenter, float rRadius, Vector3* Normal, float* Depth) const;
		bool			PenetrateCapsule(const Vector3& P0, const Vector3& P1, float rRadius, Vector3* Normal, float* Depth) const;

		bool			CalculateVolumeProperties(MassParameters* p, float Density) const
		{
			p->Volume = CalculateVolume();
			p->Mass = p->Volume * Density;
			p->BoundingVolume = CalculateBoundingVolume();
			p->CenterOfMass = p->BoundingVolume.GetCenter();
			p->InertiaMat = CalculateInertiaTensor(p->Mass);
			return true;
		}

		float			CalculateVolume() const
		{
			return CalculateVolume(Radius, GetHeight());
		}

		static float	CalculateVolume(float Radius, float Height)
		{
			return (float)M_PI * Radius * Radius * (Height + 4.0f * Radius / 3.0f);
		}

		Matrix3			CalculateInertiaTensor(float Mass) const
		{
			return CalculateInertiaTensor(Radius, GetHeight(), Mass);
		}

		static Matrix3	CalculateInertiaTensor(float Radius, float Height, float Mass)
		{
			// https://www.wolframalpha.com/input/?i=capsule&assumption=%7B%22C%22,+%22capsule%22%7D+-%3E+%7B%22Solid%22%7D
			float R = Radius;
			float H = Height;
			float RR = R * R;
			float HH = H * H;

			// (5H^3 + 20*H^2R + 45HR^2 + 32R^3) / (60H + 80R)
			float Diag12 = Mass * (5.0f * HH * H + 20.0f * HH * R + 45.0f * H * RR + 32.0f * RR * R) / (60.0f * H + 80.0f * R);
			// (R^2 * (15H + 16R) / (30H +40R))
			float Diag3 = Mass * (RR * (15.0f * H + 16.0f * R)) / (30.0f * H + 40.0f * R);

			return Matrix3(Diag12, Diag12, Diag3);
		}

		Vector3			GetSupport(const Vector3& Direction) const
		{
			return GetSupport(X0, X1, Radius, Direction);
		}

		static Vector3	GetSupport(const Vector3& X0, const Vector3& X1, float Radius, const Vector3& Direction);
		int				GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const;

		void			GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals);
		void			GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices);

	private:
		void			GetVertices(int stackCount, int sliceCount, std::vector<Vector3>& Vertices, std::vector<Vector3>* Normals);
	};
}