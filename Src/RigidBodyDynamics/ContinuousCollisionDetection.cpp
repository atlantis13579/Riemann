#include "ContinuousCollisionDetection.h"

#include <algorithm>
#include <cmath>
#include <float.h>

#include "RigidBody.h"
#include "../Collision/GeometryObject.h"
#include "../CollisionPrimitive/GJK.h"
#include "../CollisionPrimitive/MinkowskiSum.h"

namespace Riemann
{
	namespace
	{
		bool IsConvexShape(PrimitiveType type)
		{
			return type == PrimitiveType::BOX
				|| type == PrimitiveType::SPHERE
				|| type == PrimitiveType::CAPSULE
				|| type == PrimitiveType::CYLINDER
				|| type == PrimitiveType::CONVEX_MESH;
		}

		Transform GetBodyTransform(const RigidBody* body)
		{
			return Transform(body->BodyLocalToWorld(Vector3::Zero()), body->Q);
		}

		Quaternion PredictRotation(const RigidBody* body, float time)
		{
			const float angularSpeed = body->W.Length();
			if (angularSpeed <= 1.0e-8f || time <= 0.0f)
			{
				return body->Q;
			}
			if (body->mRigidType == RigidType::Dynamic
				&& static_cast<const RigidBodyDynamic*>(body)->mMotionType != MorionType::Continuous)
			{
				const Quaternion derivative = 0.5f * Quaternion(0.0f, body->W) * body->Q;
				return (body->Q + derivative * time).Unit();
			}
			Quaternion rotation;
			rotation.FromRotationAxis(body->W / angularSpeed, angularSpeed * time);
			return (rotation * body->Q).Unit();
		}

		Transform PredictGeometryTransform(const RigidBody* body, const Geometry* shape, float time)
		{
			const Quaternion bodyRotation = PredictRotation(body, time);
			const Vector3 bodyPosition = body->X + body->V * time;
			const Transform* localTransform = shape->GetTransform();
			return Transform(bodyPosition + bodyRotation * (localTransform->pos - body->CM),
				bodyRotation * localTransform->quat);
		}

		Transform PredictBodyTransform(const RigidBody* body, float time)
		{
			const Quaternion bodyRotation = PredictRotation(body, time);
			const Vector3 centerOfMass = body->X + body->V * time;
			return Transform(centerOfMass - bodyRotation * body->CM, bodyRotation);
		}

		float GetAngularRadius(const RigidBody* body, const Geometry* shape)
		{
			const Box3& bounds = shape->GetShapeBounds();
			const Vector3 bodyLocalCenter = shape->GetTransform()->LocalToWorld(bounds.GetCenter());
			return (bodyLocalCenter - body->CM).Length() + bounds.GetExtent().Length();
		}

		float GetShapeSkinWidth(const Geometry* shape)
		{
			const Vector3 extent = shape->GetShapeBounds().GetExtent();
			return std::max(0.001f, std::min(extent.x, std::min(extent.y, extent.z)));
		}

		Vector3 GetSupportPoint(const Geometry* shape, const RigidBody* body, float time, const Vector3& worldDirection)
		{
			const Transform bodyTransform = PredictBodyTransform(body, time);
			SupportFace supportFace;
			shape->GetSupportFace(bodyTransform.WorldToLocalDirection(worldDirection), supportFace);
			if (supportFace.size() <= 0)
			{
				return bodyTransform.LocalToWorld(shape->GetSupport(
					bodyTransform.WorldToLocalDirection(worldDirection)));
			}
			Vector3 supportCenter = Vector3::Zero();
			for (int index = 0; index < supportFace.size(); ++index)
			{
				supportCenter += supportFace[index];
			}
			return bodyTransform.LocalToWorld(supportCenter / (float)supportFace.size());
		}

		Vector3 GetPointVelocity(const RigidBody* body, const Vector3& worldPoint, float time)
		{
			const Vector3 centerOfMass = body->X + body->V * time;
			return body->V + body->W.Cross(worldPoint - centerOfMass);
		}

		bool EstimateSeparationDepth(Geometry* shape, const Transform& shapeTransform,
			Geometry* otherShape, const Transform& otherTransform, const Vector3& direction, float* depth)
		{
			const Vector3 unitDirection = direction.SafeUnit();
			if (unitDirection.IsZero())
			{
				return false;
			}
			float lower = 0.0f;
			float upper = 1.0e-4f;
			Transform testTransform = shapeTransform;
			testTransform.pos += unitDirection * upper;
			while (upper < 1.0f && shape->Intersect(testTransform, otherShape, otherTransform))
			{
				lower = upper;
				upper *= 2.0f;
				testTransform = shapeTransform;
				testTransform.pos += unitDirection * upper;
			}
			if (shape->Intersect(testTransform, otherShape, otherTransform))
			{
				return false;
			}
			for (int iteration = 0; iteration < 10; ++iteration)
			{
				const float middle = (lower + upper) * 0.5f;
				testTransform = shapeTransform;
				testTransform.pos += unitDirection * middle;
				if (shape->Intersect(testTransform, otherShape, otherTransform))
				{
					lower = middle;
				}
				else
				{
					upper = middle;
				}
			}
			*depth = upper;
			return true;
		}

		Vector3 EstimateAngularContactNormal(Geometry* shape, RigidBody* body, const Transform& shapeTransform,
			Geometry* otherShape, RigidBody* otherBody, const Transform& otherTransform, float time)
		{
			if (otherShape->GetShapeType() == PrimitiveType::PLANE)
			{
				return otherTransform.LocalToWorldDirection(Vector3::UnitY()).SafeUnit();
			}

			Vector3 directions[8] = {
				otherTransform.LocalToWorldDirection(Vector3::UnitX()),
				-otherTransform.LocalToWorldDirection(Vector3::UnitX()),
				otherTransform.LocalToWorldDirection(Vector3::UnitY()),
				-otherTransform.LocalToWorldDirection(Vector3::UnitY()),
				otherTransform.LocalToWorldDirection(Vector3::UnitZ()),
				-otherTransform.LocalToWorldDirection(Vector3::UnitZ()),
				(shapeTransform.pos - otherTransform.pos).SafeUnit(),
				(otherTransform.pos - shapeTransform.pos).SafeUnit()
			};
			Vector3 bestNormal = directions[6];
			float bestDepth = FLT_MAX;
			bool bestIsApproaching = false;
			for (const Vector3& direction : directions)
			{
				const Vector3 normal = direction.SafeUnit();
				float depth = 0.0f;
				if (normal.IsZero() || !EstimateSeparationDepth(shape, shapeTransform,
					otherShape, otherTransform, normal, &depth))
				{
					continue;
				}
				const Vector3 contactPoint = GetSupportPoint(shape, body, time, -normal);
				const float normalSpeed = (GetPointVelocity(body, contactPoint, time)
					- GetPointVelocity(otherBody, contactPoint, time)).Dot(normal);
				const bool approaching = normalSpeed < -1.0e-4f;
				if ((approaching && !bestIsApproaching)
					|| (approaching == bestIsApproaching && depth < bestDepth))
				{
					bestNormal = normal;
					bestDepth = depth;
					bestIsApproaching = approaching;
				}
			}
			if (bestNormal.IsZero())
			{
				bestNormal = otherTransform.LocalToWorldDirection(Vector3::UnitY()).SafeUnit();
			}
			return bestNormal;
		}

		Matrix3 GetInverseInertiaAtTime(RigidBodyDynamic* body, float time)
		{
			if (body == nullptr || body->Freezing)
			{
				return Matrix3::Zero();
			}
			const Matrix3 rotation = PredictRotation(body, time).ToRotationMatrix3();
			return rotation * body->InvInertia * rotation.Transpose();
		}

		void ApplyContactImpulse(RigidBodyDynamic* body, const Vector3& impulse,
			const Vector3& relativePosition, const Matrix3& inverseInertia)
		{
			if (body == nullptr || body->Freezing)
			{
				return;
			}
			body->V += impulse * body->InvMass;
			body->W += inverseInertia * relativePosition.Cross(impulse);
			body->Wakeup();
		}

		bool SweepAngularPair(Geometry* shape, RigidBody* body, Geometry* otherShape, RigidBody* otherBody,
			float dt, ContinuousCollisionHit* hit)
		{
			RigidBodyDynamic* otherDynamic = otherBody ? otherBody->CastDynamic() : nullptr;
			const float angularTravel = (body->W.Length()
				+ (otherDynamic && !otherDynamic->Sleeping ? otherDynamic->W.Length() : 0.0f)) * dt;
			if (angularTravel <= 1.0e-5f)
			{
				return false;
			}
			const Transform startShapeTransform = PredictGeometryTransform(body, shape, 0.0f);
			const Transform startOtherTransform = PredictGeometryTransform(otherBody, otherShape, 0.0f);
			if (shape->Intersect(startShapeTransform, otherShape, startOtherTransform))
			{
				return false;
			}

			const float bodyAngularArc = body->W.Length() * dt * GetAngularRadius(body, shape);
			const float otherAngularArc = otherDynamic && !otherDynamic->Sleeping
				? otherDynamic->W.Length() * dt * GetAngularRadius(otherBody, otherShape) : 0.0f;
			const Vector3 otherVelocity = otherDynamic && !otherDynamic->Sleeping ? otherDynamic->V : Vector3::Zero();
			const float relativeLinearTravel = ((body->V - otherVelocity) * dt).Length();
			const float sampleSpacing = std::max(0.001f, std::min(0.01f, GetShapeSkinWidth(shape) * 0.5f));
			const int displacementSamples = (int)std::ceil(
				(bodyAngularArc + otherAngularArc + relativeLinearTravel) / sampleSpacing);
			const int angularSamples = (int)std::ceil(angularTravel / 0.025f);
			const int sampleCount = std::max(1, std::min(512, std::max(displacementSamples, angularSamples)));
			float previousFraction = 0.0f;
			for (int sample = 1; sample <= sampleCount; ++sample)
			{
				const float fraction = (float)sample / (float)sampleCount;
				const Transform shapeTransform = PredictGeometryTransform(body, shape, dt * fraction);
				const Transform otherTransform = PredictGeometryTransform(otherBody, otherShape, dt * fraction);
				if (!shape->Intersect(shapeTransform, otherShape, otherTransform))
				{
					previousFraction = fraction;
					continue;
				}

				float lower = previousFraction;
				float upper = fraction;
				for (int iteration = 0; iteration < 10; ++iteration)
				{
					const float middle = (lower + upper) * 0.5f;
					const Transform middleShape = PredictGeometryTransform(body, shape, dt * middle);
					const Transform middleOther = PredictGeometryTransform(otherBody, otherShape, dt * middle);
					if (shape->Intersect(middleShape, otherShape, middleOther))
					{
						upper = middle;
					}
					else
					{
						lower = middle;
					}
				}

				const Transform impactShape = PredictGeometryTransform(body, shape, dt * upper);
				const Transform impactOther = PredictGeometryTransform(otherBody, otherShape, dt * upper);
				const float impactTime = dt * upper;
				const Vector3 normal = EstimateAngularContactNormal(shape, body, impactShape,
					otherShape, otherBody, impactOther, impactTime);
				hit->Fraction = upper;
				hit->Normal = normal;
				hit->Position = GetSupportPoint(shape, body, impactTime, -normal);
				hit->AngularHit = true;
				return true;
			}
			return false;
		}

		bool SweepConvexPair(Geometry* shape, RigidBody* body, Geometry* otherShape, RigidBody* otherBody, const Vector3& relativeDisplacement, ContinuousCollisionHit* hit)
		{
			const Transform otherTransform = GetBodyTransform(otherBody);
			const Transform initialTransform = GetBodyTransform(body);
			Vector3 normal = -relativeDisplacement.SafeUnit();
			float fraction = 0.0f;
			for (int iteration = 0; iteration < 32 && fraction <= 1.0f; ++iteration)
			{
				Transform movingTransform = initialTransform;
				movingTransform.pos += relativeDisplacement * fraction;
				TransformedMinkowskiSum<Geometry, Geometry> difference(
					otherShape, otherTransform,
					shape, movingTransform);
				GJKIntersection gjk;
				const GJK_status status = gjk.Solve(&difference);
				if (status == GJK_status::Intersect || (status == GJK_status::NotIntersect && gjk.distance <= 5e-4f))
				{
					hit->Fraction = fraction;
					hit->Normal = normal;
					hit->Position = body->GetGeometryTransform(shape).pos + relativeDisplacement * fraction;
					return true;
				}
				if (status != GJK_status::NotIntersect || gjk.distance == FLT_MAX)
				{
					return false;
				}

				const Vector3 separationDirection = gjk.closest.SafeUnit();
				const float closingDistance = relativeDisplacement.Dot(separationDirection);
				if (closingDistance <= 1e-6f)
				{
					return false;
				}
				normal = -separationDirection;
				const float advance = std::max(1e-5f, (gjk.distance - 2.5e-4f) / closingDistance);
				fraction += advance;
			}
			return false;
		}

		bool SweepConcaveTarget(Geometry* shape, RigidBody* body, Geometry* otherShape, RigidBody* otherBody, const Vector3& relativeDisplacement, ContinuousCollisionHit* hit)
		{
			const Transform shapeTransform = body->GetGeometryTransform(shape);
			const Transform otherTransform = otherBody->GetGeometryTransform(otherShape);
			const Vector3 localDirection = shapeTransform.WorldToLocalDirection(relativeDisplacement);
			Vector3 localPosition;
			Vector3 localNormal;
			float fraction = FLT_MAX;
			if (!shape->Sweep(shapeTransform, localDirection, otherShape, otherTransform, &localPosition, &localNormal, &fraction))
			{
				return false;
			}
			if (fraction < 0.0f || fraction > 1.0f)
			{
				return false;
			}

			hit->Fraction = fraction;
			hit->Normal = shapeTransform.LocalToWorldDirection(localNormal).SafeUnit();
			hit->Position = shapeTransform.LocalToWorld(localPosition);
			return true;
		}

		bool SweepPair(Geometry* shape, RigidBody* body, Geometry* otherShape, RigidBody* otherBody, const Vector3& relativeDisplacement, ContinuousCollisionHit* hit)
		{
			if (IsConvexShape(otherShape->GetShapeType()))
			{
				return SweepConvexPair(shape, body, otherShape, otherBody, relativeDisplacement, hit);
			}
			return SweepConcaveTarget(shape, body, otherShape, otherBody, relativeDisplacement, hit);
		}
	}

	ContinuousCollisionHit::ContinuousCollisionHit()
		: Body(nullptr)
		, OtherBody(nullptr)
		, Shape(nullptr)
		, OtherShape(nullptr)
		, Position(Vector3::Zero())
		, Normal(Vector3::Zero())
		, Fraction(1.0f)
		, AngularHit(false)
	{
	}

	bool ContinuousCollisionDetection::IsSupportedMovingShape(const Geometry* shape)
	{
		return shape != nullptr && IsConvexShape(shape->GetShapeType());
	}

	bool ContinuousCollisionDetection::FindEarliestHit(RigidBodyDynamic* body, const std::vector<RigidBody*>& bodies, float dt, ContinuousCollisionHit* hit)
	{
		if (body == nullptr || hit == nullptr || body->mMotionType != MorionType::Continuous || body->Sleeping)
		{
			return false;
		}

		const Vector3 bodyDisplacement = body->V * dt;
		if (bodyDisplacement.SquareLength() <= 1e-10f && body->W.SquareLength() * dt * dt <= 1e-10f)
		{
			return false;
		}

		ContinuousCollisionHit best;
		for (Geometry* shape : body->Geometries())
		{
			if (!shape || !shape->IsSimulationEnabled() || shape->IsTriggerEnabled() || !IsSupportedMovingShape(shape))
			{
				continue;
			}

			for (RigidBody* otherBody : bodies)
			{
				if (otherBody == nullptr || !body->CanCollideWith(otherBody))
				{
					continue;
				}
				RigidBodyDynamic* otherDynamic = otherBody->CastDynamic();
				const Vector3 otherDisplacement = otherDynamic && !otherDynamic->Sleeping ? otherDynamic->V * dt : Vector3::Zero();
				const Vector3 relativeDisplacement = bodyDisplacement - otherDisplacement;
				for (Geometry* otherShape : otherBody->Geometries())
				{
					if (!otherShape || !otherShape->IsSimulationEnabled() || otherShape->IsTriggerEnabled())
					{
						continue;
					}

					ContinuousCollisionHit candidate;
					bool foundCandidate = relativeDisplacement.SquareLength() > 1e-10f
						&& SweepPair(shape, body, otherShape, otherBody, relativeDisplacement, &candidate);
					ContinuousCollisionHit angularCandidate;
					if (SweepAngularPair(shape, body, otherShape, otherBody, dt, &angularCandidate)
						&& (!foundCandidate || angularCandidate.Fraction < candidate.Fraction))
					{
						candidate = angularCandidate;
						foundCandidate = true;
					}
					if (!foundCandidate)
					{
						continue;
					}
					if (candidate.Fraction <= 1e-5f || candidate.Fraction >= best.Fraction)
					{
						continue;
					}

					candidate.Body = body;
					candidate.OtherBody = otherBody;
					candidate.Shape = shape;
					candidate.OtherShape = otherShape;
					best = candidate;
				}
			}
		}

		if (best.Body == nullptr)
		{
			return false;
		}
		*hit = best;
		return true;
	}

	void ContinuousCollisionDetection::FindTriggerHits(const std::vector<RigidBody*>& bodies, float dt,
		std::vector<ContinuousCollisionHit>* hits)
	{
		if (hits == nullptr)
		{
			return;
		}
		hits->clear();
		for (size_t bodyIndexA = 0; bodyIndexA < bodies.size(); ++bodyIndexA)
		{
			RigidBody* bodyA = bodies[bodyIndexA];
			if (bodyA == nullptr)
			{
				continue;
			}
			for (size_t bodyIndexB = bodyIndexA + 1; bodyIndexB < bodies.size(); ++bodyIndexB)
			{
				RigidBody* bodyB = bodies[bodyIndexB];
				if (bodyB == nullptr || !bodyA->CanCollideWith(bodyB))
				{
					continue;
				}
				RigidBodyDynamic* dynamicA = bodyA->CastDynamic();
				RigidBodyDynamic* dynamicB = bodyB->CastDynamic();
				const Vector3 displacementA = dynamicA && !dynamicA->Sleeping ? dynamicA->V * dt : Vector3::Zero();
				const Vector3 displacementB = dynamicB && !dynamicB->Sleeping ? dynamicB->V * dt : Vector3::Zero();
				const Vector3 relativeDisplacement = displacementA - displacementB;

				for (Geometry* shapeA : bodyA->Geometries())
				{
					if (shapeA == nullptr || (!shapeA->IsSimulationEnabled() && !shapeA->IsTriggerEnabled()))
					{
						continue;
					}
					for (Geometry* shapeB : bodyB->Geometries())
					{
						if (shapeB == nullptr || (!shapeB->IsSimulationEnabled() && !shapeB->IsTriggerEnabled()))
						{
							continue;
						}
						const bool triggerPair = shapeA->IsTriggerEnabled() || shapeB->IsTriggerEnabled();
						const bool ccdEnabled = (shapeA->IsTriggerEnabled() && shapeA->IsTriggerCCDEnabled())
							|| (shapeB->IsTriggerEnabled() && shapeB->IsTriggerCCDEnabled());
						if (!triggerPair || !ccdEnabled)
						{
							continue;
						}

						ContinuousCollisionHit candidate;
						bool reversed = false;
						bool found = false;
						if (relativeDisplacement.SquareLength() > 1.0e-10f)
						{
							if (IsSupportedMovingShape(shapeA))
							{
								found = SweepPair(shapeA, bodyA, shapeB, bodyB, relativeDisplacement, &candidate);
							}
							else if (IsSupportedMovingShape(shapeB))
							{
								found = SweepPair(shapeB, bodyB, shapeA, bodyA, -relativeDisplacement, &candidate);
								reversed = true;
							}
						}

						ContinuousCollisionHit angularCandidate;
						bool angularReversed = false;
						bool angularFound = false;
						if (dynamicA && !dynamicA->Sleeping && IsSupportedMovingShape(shapeA))
						{
							angularFound = SweepAngularPair(shapeA, bodyA, shapeB, bodyB, dt, &angularCandidate);
						}
						else if (dynamicB && !dynamicB->Sleeping && IsSupportedMovingShape(shapeB))
						{
							angularFound = SweepAngularPair(shapeB, bodyB, shapeA, bodyA, dt, &angularCandidate);
							angularReversed = true;
						}
						if (angularFound && (!found || angularCandidate.Fraction < candidate.Fraction))
						{
							candidate = angularCandidate;
							found = true;
							reversed = angularReversed;
						}
						if (!found || candidate.Fraction <= 1.0e-5f || candidate.Fraction > 1.0f)
						{
							continue;
						}

						candidate.Body = bodyA;
						candidate.OtherBody = bodyB;
						candidate.Shape = shapeA;
						candidate.OtherShape = shapeB;
						if (reversed)
						{
							candidate.Normal = -candidate.Normal;
						}
						hits->push_back(candidate);
					}
				}
			}
		}
	}

	int ContinuousCollisionDetection::Resolve(std::vector<RigidBodyDynamic*>& dynamicBodies, const std::vector<RigidBody*>& bodies, float dt)
	{
		for (RigidBodyDynamic* body : dynamicBodies)
		{
			body->CCDMotionFraction = 1.0f;
			body->HadCCDContact = false;
			body->CCDPreSolveVelocity = body->V;
			body->CCDPreSolveAngularVelocity = body->W;
		}

		int hitCount = 0;
		for (RigidBodyDynamic* body : dynamicBodies)
		{
			ContinuousCollisionHit hit;
			if (!FindEarliestHit(body, bodies, dt, &hit))
			{
				continue;
			}

			RigidBodyDynamic* otherDynamic = hit.OtherBody->CastDynamic();
			const float impactTime = dt * hit.Fraction;
			Vector3 normal = hit.Normal.SafeUnit();
			if (normal.IsZero())
			{
				normal = -(body->V - (otherDynamic ? otherDynamic->V : Vector3::Zero())).SafeUnit();
			}
			Vector3 contactPosition = GetSupportPoint(hit.Shape, body, impactTime, -normal);
			Vector3 relativePointVelocity = GetPointVelocity(body, contactPosition, impactTime)
				- GetPointVelocity(hit.OtherBody, contactPosition, impactTime);
			if (relativePointVelocity.Dot(normal) > 0.0f)
			{
				normal = -normal;
				contactPosition = GetSupportPoint(hit.Shape, body, impactTime, -normal);
				relativePointVelocity = GetPointVelocity(body, contactPosition, impactTime)
					- GetPointVelocity(hit.OtherBody, contactPosition, impactTime);
			}
			const float contactPathLength = relativePointVelocity.Length() * dt;
			const float backoff = contactPathLength > 1e-5f
				? std::min(0.01f, 0.001f / contactPathLength) : 0.0f;
			body->CCDMotionFraction = std::max(0.0f, hit.Fraction - backoff);
			body->HadCCDContact = true;

			const float normalSpeed = relativePointVelocity.Dot(normal);
			if (normalSpeed < 0.0f)
			{
				const Vector3 centerA = body->X + body->V * impactTime;
				const Vector3 centerB = hit.OtherBody->X + hit.OtherBody->V * impactTime;
				const Vector3 relativePositionA = contactPosition - centerA;
				const Vector3 relativePositionB = contactPosition - centerB;
				const Matrix3 inverseInertiaA = GetInverseInertiaAtTime(body, impactTime);
				const Matrix3 inverseInertiaB = GetInverseInertiaAtTime(otherDynamic, impactTime);
				const float inverseMassA = body->Freezing ? 0.0f : body->InvMass;
				const float inverseMassB = otherDynamic && !otherDynamic->Freezing ? otherDynamic->InvMass : 0.0f;
				const float angularWeightA = std::max(0.0f, normal.Dot(
					(inverseInertiaA * relativePositionA.Cross(normal)).Cross(relativePositionA)));
				const float angularWeightB = std::max(0.0f, normal.Dot(
					(inverseInertiaB * relativePositionB.Cross(normal)).Cross(relativePositionB)));
				const float effectiveInverseMass = inverseMassA + inverseMassB + angularWeightA + angularWeightB;
				if (effectiveInverseMass > kMinimumInvMass)
				{
					const Vector3 impulse = normal * (-normalSpeed / effectiveInverseMass);
					ApplyContactImpulse(body, impulse, relativePositionA, inverseInertiaA);
					ApplyContactImpulse(otherDynamic, -impulse, relativePositionB, inverseInertiaB);
				}
			}
			if (otherDynamic && otherDynamic->mMotionType == MorionType::Continuous)
			{
				otherDynamic->CCDMotionFraction = std::min(otherDynamic->CCDMotionFraction, body->CCDMotionFraction);
				otherDynamic->HadCCDContact = true;
			}
			++hitCount;
		}
		return hitCount;
	}
}
