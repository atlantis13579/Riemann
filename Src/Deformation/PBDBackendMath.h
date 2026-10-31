#pragma once

#include <cfloat>
#include <cmath>

#include "PBDBackend.h"

#if defined(__CUDACC__)
#define RIEMANN_PBD_HD __host__ __device__
#else
#define RIEMANN_PBD_HD
#endif

namespace Riemann
{
	namespace PBDBackendMath
	{
		using Vector = PBDBackendVector;
		RIEMANN_PBD_HD inline Vector Make(float X, float Y, float Z, float W = 0.0f) { return { X, Y, Z, W }; }
		RIEMANN_PBD_HD inline float Min(float A, float B) { return A < B ? A : B; }
		RIEMANN_PBD_HD inline float Max(float A, float B) { return A > B ? A : B; }
		RIEMANN_PBD_HD inline bool Finite(float Value) { return Value == Value && Value >= -FLT_MAX && Value <= FLT_MAX; }
		RIEMANN_PBD_HD inline void StoreXYZ(Vector& Target, const Vector& Value)
		{
			Target.x = Value.x; Target.y = Value.y; Target.z = Value.z;
		}

		struct Scalar
		{
			RIEMANN_PBD_HD static Vector Add(const Vector& A, const Vector& B) { return Make(A.x + B.x, A.y + B.y, A.z + B.z); }
			RIEMANN_PBD_HD static Vector Sub(const Vector& A, const Vector& B) { return Make(A.x - B.x, A.y - B.y, A.z - B.z); }
			RIEMANN_PBD_HD static Vector Mul(const Vector& A, float B) { return Make(A.x * B, A.y * B, A.z * B); }
			RIEMANN_PBD_HD static float Dot(const Vector& A, const Vector& B) { return (A.x * B.x + A.y * B.y) + A.z * B.z; }
			RIEMANN_PBD_HD static Vector Cross(const Vector& A, const Vector& B)
			{
				return Make(A.y * B.z - A.z * B.y, A.z * B.x - A.x * B.z, A.x * B.y - A.y * B.x);
			}
		};

		template<class Math>
		RIEMANN_PBD_HD Vector RotateVector(const Vector& Q, const Vector& Value)
		{
			const Vector TwiceCross = Math::Mul(Math::Cross(Q, Value), 2.0f);
			return Math::Add(Math::Add(Value, Math::Mul(TwiceCross, Q.w)), Math::Cross(Q, TwiceCross));
		}

		template<class Math>
		RIEMANN_PBD_HD Vector InverseInertia(const PBDBackendBody& Body, const Vector& Value)
		{
			if (!Body.Flags.x) return Make(0, 0, 0);
			const Vector InverseQ = Make(-Body.Rotation.x, -Body.Rotation.y, -Body.Rotation.z, Body.Rotation.w);
			const Vector Local = RotateVector<Math>(InverseQ, Value);
			const Vector Result = Make(Math::Dot(Body.InverseInertia0, Local),
				Math::Dot(Body.InverseInertia1, Local), Math::Dot(Body.InverseInertia2, Local));
			return RotateVector<Math>(Body.Rotation, Result);
		}

		template<class Math>
		RIEMANN_PBD_HD void RotateBody(PBDBackendBody& Body, const Vector& Angular)
		{
			const float Angle = sqrtf(Math::Dot(Angular, Angular));
			if (Angle <= 1.0e-8f) return;
			Vector Axis = Make(Angular.x / Angle, Angular.y / Angle, Angular.z / Angle);
			const float AxisLength = sqrtf(Math::Dot(Axis, Axis));
			Axis = Make(Axis.x / AxisLength, Axis.y / AxisLength, Axis.z / AxisLength);
			const float Sine = sinf(Angle * 0.5f);
			const Vector Delta = Make(Axis.x * Sine, Axis.y * Sine, Axis.z * Sine, cosf(Angle * 0.5f));
			Vector Result = Math::Add(Math::Add(Math::Cross(Delta, Body.Rotation), Math::Mul(Delta, Body.Rotation.w)),
				Math::Mul(Body.Rotation, Delta.w));
			Result.w = ((Delta.w * Body.Rotation.w - Delta.x * Body.Rotation.x) - Delta.y * Body.Rotation.y) - Delta.z * Body.Rotation.z;
			const float Length = sqrtf(((Result.w * Result.w + Result.x * Result.x) + Result.y * Result.y) + Result.z * Result.z);
			Body.Rotation = Make(Result.x / Length, Result.y / Length, Result.z / Length, Result.w / Length);
		}

		template<class Math>
		RIEMANN_PBD_HD void ApplyPosition(PBDBackendBody& Body, const Vector& Impulse, const Vector& Offset)
		{
			if (!Body.Flags.x || Math::Dot(Impulse, Impulse) <= 1.0e-20f) return;
			const Vector Angular = InverseInertia<Math>(Body, Math::Cross(Offset, Impulse));
			StoreXYZ(Body.Position, Math::Add(Body.Position, Math::Mul(Impulse, Body.Position.w)));
			RotateBody<Math>(Body, Angular);
			Body.Flags.z = 1;
		}

		template<class Math>
		RIEMANN_PBD_HD void ApplyVelocity(PBDBackendBody& Body, const Vector& Impulse, const Vector& Offset)
		{
			if (!Body.Flags.x || Math::Dot(Impulse, Impulse) <= 1.0e-20f) return;
			StoreXYZ(Body.LinearVelocity, Math::Add(Body.LinearVelocity, Math::Mul(Impulse, Body.Position.w)));
			StoreXYZ(Body.AngularVelocity, Math::Add(Body.AngularVelocity, InverseInertia<Math>(Body, Math::Cross(Offset, Impulse))));
			Body.Flags.z = 1;
		}

		template<class Math>
		RIEMANN_PBD_HD Vector PointVelocity(const PBDBackendBody& Body, const Vector& Offset)
		{
			return Body.Flags.y ? Math::Add(Body.LinearVelocity, Math::Cross(Body.AngularVelocity, Offset)) : Make(0, 0, 0);
		}

		template<class Math>
		RIEMANN_PBD_HD float Weight(const PBDBackendBody& A, const PBDBackendBody& B,
			const Vector& RA, const Vector& RB, const Vector& Axis)
		{
			const Vector AngularA = Math::Cross(RA, Axis), AngularB = Math::Cross(RB, Axis);
			return A.Position.w + B.Position.w + Math::Dot(AngularA, InverseInertia<Math>(A, AngularA))
				+ Math::Dot(AngularB, InverseInertia<Math>(B, AngularB));
		}

		template<class Math>
		RIEMANN_PBD_HD void SolveRow(PBDBackendBody* Bodies, PBDBackendContact& Contact,
			const PBDBackendParameters& Parameters, PBDBackendStage Stage)
		{
			PBDBackendBody& A = Bodies[Contact.Bodies.x];
			PBDBackendBody& B = Bodies[Contact.Bodies.y];
			const Vector RA = RotateVector<Math>(A.Rotation, Contact.AnchorA);
			const Vector RB = RotateVector<Math>(B.Rotation, Contact.AnchorB);
			if (Stage == PBDBackendStage::Reconstruct)
			{
				const Vector Impulse = Math::Mul(Contact.Normal, Contact.Lambda.x / Parameters.DeltaTime);
				ApplyVelocity<Math>(A, Math::Mul(Impulse, -1.0f), RA);
				ApplyVelocity<Math>(B, Impulse, RB);
				return;
			}
			const float EffectiveMass = Weight<Math>(A, B, RA, RB, Contact.Normal);
			if (!Finite(EffectiveMass) || EffectiveMass <= 1.0e-9f) return;
			if (Stage == PBDBackendStage::Position)
			{
				const Vector PA = Math::Add(A.Position, RA), PB = Math::Add(B.Position, RB);
				const float Error = Math::Dot(Math::Sub(PB, PA), Contact.Normal) + Parameters.PenetrationSlop;
				float Delta = -Error / EffectiveMass;
				if (Parameters.Alpha > 0.0f)
				{
					// Normalize before adding so very soft contacts do not overflow alpha + weight.
					const float Scale = Max(EffectiveMass, Parameters.Alpha);
					const float Denominator = EffectiveMass / Scale + Parameters.Alpha / Scale;
					Delta = -(Error / Scale) / Denominator - Contact.Lambda.x * ((Parameters.Alpha / Scale) / Denominator);
				}
				Delta = Max(-Contact.Lambda.x, Delta);
				if (!Finite(Delta)) return;
				const float Translation = Max(A.Position.w, B.Position.w) * fabsf(Delta);
				const Vector AngularA = InverseInertia<Math>(A, Math::Cross(RA, Contact.Normal));
				const Vector AngularB = InverseInertia<Math>(B, Math::Cross(RB, Contact.Normal));
				const float Rotation = Max(sqrtf(Math::Dot(AngularA, AngularA)), sqrtf(Math::Dot(AngularB, AngularB))) * fabsf(Delta);
				float Scale = 1.0f;
				if (Translation > Parameters.MaxPositionCorrection) Scale = Min(Scale, Parameters.MaxPositionCorrection / Translation);
				if (Rotation > Parameters.MaxAngularCorrection) Scale = Min(Scale, Parameters.MaxAngularCorrection / Rotation);
				Delta *= Scale;
				Contact.Lambda.x += Delta;
				ApplyPosition<Math>(A, Math::Mul(Contact.Normal, -Delta), RA);
				ApplyPosition<Math>(B, Math::Mul(Contact.Normal, Delta), RB);
				return;
			}

			const float Speed = Math::Dot(Math::Sub(PointVelocity<Math>(B, RB), PointVelocity<Math>(A, RA)), Contact.Normal);
			const float PreviousImpulse = Contact.Lambda.x / Parameters.DeltaTime + Contact.Lambda.y;
			const float Limit = Min(A.LinearVelocity.w, B.LinearVelocity.w);
			const float TotalImpulse = Max(0.0f, Min(Limit, PreviousImpulse + (Contact.Material.x - Speed) / EffectiveMass));
			const float Delta = TotalImpulse - PreviousImpulse;
			Contact.Lambda.y += Delta;
			ApplyVelocity<Math>(A, Math::Mul(Contact.Normal, -Delta), RA);
			ApplyVelocity<Math>(B, Math::Mul(Contact.Normal, Delta), RB);

			const Vector Relative = Math::Sub(PointVelocity<Math>(B, RB), PointVelocity<Math>(A, RA));
			const float KT = Weight<Math>(A, B, RA, RB, Contact.Tangent);
			const float KB = Weight<Math>(A, B, RA, RB, Contact.Binormal);
			const float KTB = Math::Dot(Math::Cross(RA, Contact.Tangent), InverseInertia<Math>(A, Math::Cross(RA, Contact.Binormal)))
				+ Math::Dot(Math::Cross(RB, Contact.Tangent), InverseInertia<Math>(B, Math::Cross(RB, Contact.Binormal)));
			const float Determinant = KT * KB - KTB * KTB;
			if (!Finite(Determinant) || Determinant <= 1.0e-12f) return;
			const float VT = Math::Dot(Relative, Contact.Tangent), VB = Math::Dot(Relative, Contact.Binormal);
			float Tangent = Contact.Lambda.z - (KB * VT - KTB * VB) / Determinant;
			float Binormal = Contact.Lambda.w - (KT * VB - KTB * VT) / Determinant;
			const float Magnitude = sqrtf(Tangent * Tangent + Binormal * Binormal);
			if (Magnitude > Contact.Material.z * TotalImpulse && Magnitude > 0.0f)
			{
				const float Scale = Min(1.0f, Contact.Material.y * TotalImpulse / Magnitude);
				Tangent *= Scale; Binormal *= Scale;
			}
			const Vector Impulse = Math::Add(Math::Mul(Contact.Tangent, Tangent - Contact.Lambda.z),
				Math::Mul(Contact.Binormal, Binormal - Contact.Lambda.w));
			Contact.Lambda.z = Tangent; Contact.Lambda.w = Binormal;
			ApplyVelocity<Math>(A, Math::Mul(Impulse, -1.0f), RA);
			ApplyVelocity<Math>(B, Impulse, RB);
		}
	}
}

#undef RIEMANN_PBD_HD
