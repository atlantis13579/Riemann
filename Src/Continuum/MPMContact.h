#pragma once

#include "SnowSimulation.h"

namespace Riemann
{
	namespace MPM
	{
		RIEMANN_MPM_HD inline Vector GroundContact(Vector Velocity, float Friction)
		{
			if (Velocity.y >= 0) return Velocity;
			const float Speed = sqrtf(Velocity.x * Velocity.x + Velocity.z * Velocity.z);
			const float Scale = fmaxf(0, 1 + Friction * Velocity.y / fmaxf(Speed, 1.e-6f));
			return {Velocity.x * Scale, 0, Velocity.z * Scale};
		}
		RIEMANN_MPM_HD inline Vector CapsuleClosest(Vector Position, const SnowCapsule& Capsule)
		{ return {Capsule.Center.x, Clamp(Position.y, Capsule.Center.y - Capsule.HalfLength, Capsule.Center.y + Capsule.HalfLength), Capsule.Center.z}; }
		RIEMANN_MPM_HD inline Vector Contact(Vector Position, Vector Velocity, const SnowCapsule& Capsule, float Padding)
		{
			if (!Capsule.Enabled) return Velocity;
			const Vector Offset = Position - CapsuleClosest(Position, Capsule);
			const float Length = Offset.Length();
			if (Length >= Capsule.Radius + Padding) return Velocity;
			const Vector Normal = Length > 1.e-6f ? Offset / Length : Vector(1,0,0);
			const Vector Relative = Velocity - Capsule.Velocity;
			const float Incoming = Relative.Dot(Normal);
			if (Incoming >= 0) return Velocity;
			const Vector Tangent = Relative - Normal * Incoming;
			const float Friction = fmaxf(0, 1 - Capsule.Friction * (-Incoming) / fmaxf(Tangent.Length(), 1.e-6f));
			return Capsule.Velocity + Tangent * Friction;
		}
		RIEMANN_MPM_HD inline void Confine(Vector& Position, Vector& Velocity, const SnowSettings& Settings, const SnowCapsule& Capsule)
		{
			for (int A = 0; A < 3; ++A)
			{
				const float Low = Settings.BoundsMin[A] + 1.5f * Settings.CellSize, High = Settings.BoundsMax[A] - 1.5f * Settings.CellSize;
				if (Position[A] < Low || Position[A] > High) Velocity[A] = 0;
				Position[A] = Clamp(Position[A], Low, High);
			}
			Velocity = Contact(Position, Velocity, Capsule, 0);
			if (Capsule.Enabled)
			{
				const Vector Closest = CapsuleClosest(Position, Capsule), Offset = Position - Closest;
				const float Length = Offset.Length();
				if (Length < Capsule.Radius) Position = Closest + (Length > 1.e-6f ? Offset / Length : Vector(1,0,0)) * Capsule.Radius;
			}
			for (int A = 0; A < 3; ++A)
				Position[A] = Clamp(Position[A], Settings.BoundsMin[A] + 1.5f * Settings.CellSize, Settings.BoundsMax[A] - 1.5f * Settings.CellSize);
		}
	}
}
