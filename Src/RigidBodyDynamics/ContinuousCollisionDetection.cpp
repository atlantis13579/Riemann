#include "ContinuousCollisionDetection.h"

#include <algorithm>
#include <cmath>
#include <float.h>
#include <stdexcept>
#include <unordered_map>
#include "Constraints.h"
#include "MeshContactGenerator.h"

#include "RigidBody.h"
#include "../Collision/GeometryObject.h"
#include "../Collision/GeometryQuery.h"
#include "../Collision/DistanceFieldCollision.h"
#include "../CollisionPrimitive/DistanceFieldShape.h"
#include "../CollisionPrimitive/GJK.h"
#include "../CollisionPrimitive/MinkowskiSum.h"

namespace Riemann
{
	namespace
	{
		const float MinimumAngularSweepTravel = 1.0e-5f;
		bool NeedsFieldAngularSweep(const Geometry* Shape, RigidBody* Body, const Geometry* Other, RigidBody* OtherBody, float Dt)
		{
			if (!DistanceFieldCollision::HasField(Shape, Other)) return false;
			const auto* DynamicA = Body->CastDynamic(); const auto* DynamicB = OtherBody->CastDynamic();
			const float SpeedA = DynamicA && !DynamicA->Sleeping ? DynamicA->W.Length() : 0.0f;
			const float SpeedB = DynamicB && !DynamicB->Sleeping ? DynamicB->W.Length() : 0.0f;
			return (SpeedA + SpeedB) * Dt > MinimumAngularSweepTravel;
		}

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
			// Kinematic poses are prescribed snapshots, not integrated from V/W.
			// Their velocities still enter contact response; do not move them twice.
			if (body->mRigidType != RigidType::Dynamic) return body->Q;
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
			const Vector3 bodyPosition = body->mRigidType == RigidType::Dynamic ? body->X + body->V * time : body->X;
			const Transform* localTransform = shape->GetTransform();
			return Transform(bodyPosition + bodyRotation * (localTransform->pos - body->CM),
				bodyRotation * localTransform->quat);
		}

		Transform PredictBodyTransform(const RigidBody* body, float time)
		{
			const Quaternion bodyRotation = PredictRotation(body, time);
			const Vector3 centerOfMass = body->mRigidType == RigidType::Dynamic ? body->X + body->V * time : body->X;
			return Transform(centerOfMass - bodyRotation * body->CM, bodyRotation);
		}

		float GetAngularRadius(const RigidBody* body, const Geometry* shape)
		{
			const Box3& bounds = shape->GetShapeBounds();
			const Vector3 bodyLocalCenter = shape->GetTransform()->LocalToWorld(bounds.GetCenter());
			return (bodyLocalCenter - body->CM).Length() + bounds.GetExtent().Length();
		}
		Box3 FieldMotionBounds(RigidBody* Body, const Geometry* Shape, float Dt)
		{
			const Transform Pose = PredictGeometryTransform(Body, Shape, 0.0f);
			Box3 Bounds = Box3::Transform(Shape->GetShapeBounds(), Pose.pos, Pose.quat);
			const auto* Dynamic = Body->CastDynamic();
			if (Dynamic && !Dynamic->Sleeping)
			{
				// Every point stays within its angular arc (or its rotation diameter).
				// Union translation endpoints before inflating for this rotation bound.
				const Vector3 Displacement = Body->V * Dt;
				Bounds.Encapsulate(Bounds.Min + Displacement, Bounds.Max + Displacement);
				Bounds.Thicken(std::min(2.0f, Body->W.Length() * Dt) * GetAngularRadius(Body, Shape));
			}
			return Bounds;
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

		bool SweepFieldAngularPair(Geometry* Shape, RigidBody* Body, Geometry* Other, RigidBody* OtherBody,
			float Dt, ContinuousCollisionHit* Hit)
		{
			const auto* Field = Shape->GetShapeObj<DistanceFieldShape>(); if (!Field) Field = Other->GetShapeObj<DistanceFieldShape>();
			const auto* OtherDynamic = OtherBody->CastDynamic();
			const Vector3 OtherVelocity = OtherDynamic && !OtherDynamic->Sleeping ? OtherBody->V : Vector3::Zero();
			const float OtherSpeed = OtherDynamic && !OtherDynamic->Sleeping ? OtherBody->W.Length() : 0.0f;
			const double Speed = (Body->V - OtherVelocity).Length() + Body->W.Length() * GetAngularRadius(Body, Shape)
				+ (OtherSpeed > 0.0f ? OtherSpeed * GetAngularRadius(OtherBody, Other) : 0.0f);
			if (!(Speed > 0.0)) return false;
			const float Tolerance = Field->GetField().GetCellSize() * Field->GetSettings().RelativeSweepTolerance;
			float Time = 0.0f;
			// Rotation moves every shape point by at most angularSpeed * radius.
			// Adding both angular bounds to relative linear speed makes the same
			// certified distance advance valid for the complete predicted poses.
			for (int Iteration = 0; Iteration < Field->GetSettings().MaximumSweepIterations; ++Iteration)
			{
				DistanceFieldPairDistance Query;
				DistanceFieldCollision::Distance(Shape, PredictGeometryTransform(Body, Shape, Time), Other, PredictGeometryTransform(OtherBody, Other, Time), Query, Tolerance / 2);
				if (Query.Status == DistanceFieldSearchStatus::InvalidInput || (!Query.WitnessValid && Query.Converged)) return false;
				const bool Close = Query.UpperBound <= Tolerance;
				const double Next = Time + Query.LowerBound / Speed;
				if (Close || Query.LowerBound <= 0.0f || Next <= Time || Iteration + 1 == Field->GetSettings().MaximumSweepIterations)
				{
					Hit->Fraction = Time / Dt; Hit->AngularHit = true; Hit->ConservativeHit = !Close;
					Hit->Position = Query.WitnessValid ? Query.PointA : Body->X + Body->V * Time;
					Hit->Normal = Query.Normal.SafeUnit();
					if (Hit->Normal.IsZero()) Hit->Normal = -(GetPointVelocity(Body, Hit->Position, Time) - GetPointVelocity(OtherBody, Hit->Position, Time)).SafeUnit();
					return true;
				}
				if (Next > Dt) return false;
				float Rounded = (float)Next; if (Rounded > Next) Rounded = std::nextafter(Rounded, 0.0f);
				if (Rounded <= Time)
				{
					Hit->Fraction = Time / Dt; Hit->AngularHit = Hit->ConservativeHit = true;
					Hit->Position = Query.WitnessValid ? Query.PointA : Body->X + Body->V * Time; Hit->Normal = Query.Normal.SafeUnit(); return true;
				}
				Time = Rounded;
			}
			return false;
		}

		bool SweepAngularPair(Geometry* shape, RigidBody* body, Geometry* otherShape, RigidBody* otherBody,
			float dt, ContinuousCollisionHit* hit, bool AllowInitialContact = false)
		{
			RigidBodyDynamic* otherDynamic = otherBody ? otherBody->CastDynamic() : nullptr;
			const float angularTravel = (body->W.Length()
				+ (otherDynamic && !otherDynamic->Sleeping ? otherDynamic->W.Length() : 0.0f)) * dt;
			if (angularTravel <= MinimumAngularSweepTravel)
			{
				return false;
			}
			if (DistanceFieldCollision::HasField(shape, otherShape)
				&& otherShape->GetShapeType() != PrimitiveType::PLANE && shape->GetShapeType() != PrimitiveType::PLANE
				&& !FieldMotionBounds(body, shape, dt).Intersect(FieldMotionBounds(otherBody, otherShape, dt))) return false;
			const Transform startShapeTransform = PredictGeometryTransform(body, shape, 0.0f);
			const Transform startOtherTransform = PredictGeometryTransform(otherBody, otherShape, 0.0f);
			if (!AllowInitialContact && shape->Intersect(startShapeTransform, otherShape, startOtherTransform))
			{
				return false;
			}

			if (DistanceFieldCollision::HasField(shape, otherShape)) return SweepFieldAngularPair(shape, body, otherShape, otherBody, dt, hit);

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
			if (DistanceFieldCollision::HasField(shape, otherShape))
			{
				const Transform Pose = body->GetGeometryTransform(shape), OtherPose = otherBody->GetGeometryTransform(otherShape);
				DistanceFieldSweepResult Result;
				if (!DistanceFieldCollision::SweepBounded(shape, Pose, otherShape, OtherPose,
					Pose.WorldToLocalDirection(relativeDisplacement), 1.0f, Result) || Result.InitialOverlap) return false;
				hit->Fraction = Result.Time; hit->Position = Pose.LocalToWorld(Result.Position);
				hit->Normal = Pose.LocalToWorldDirection(Result.Normal); hit->ConservativeHit = Result.Conservative; return true;
			}
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
		, ConservativeHit(false)
	{
	}

	bool ContinuousCollisionDetection::IsSupportedMovingShape(const Geometry* shape)
	{
		return shape != nullptr && (IsConvexShape(shape->GetShapeType()) || shape->GetShapeType() == PrimitiveType::DISTANCE_FIELD);
	}

	bool ContinuousCollisionDetection::FindEarliestHit(RigidBodyDynamic* body, const std::vector<RigidBody*>& bodies, float dt, ContinuousCollisionHit* hit, CollisionFilter* Filter)
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
					if (!otherShape || !otherShape->IsSimulationEnabled() || otherShape->IsTriggerEnabled()
						|| !CollisionFilter::Allows(Filter, shape->GetFilterData(), otherShape->GetFilterData()))
					{
						continue;
					}

					ContinuousCollisionHit candidate;
					// The field angular query already follows both full poses. A sweep
					// at the initial orientation can report an obstacle rotated away from.
					bool foundCandidate = !NeedsFieldAngularSweep(shape, body, otherShape, otherBody, dt)
						&& relativeDisplacement.SquareLength() > 1e-10f
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
					if ((candidate.Fraction <= 1e-5f && !candidate.ConservativeHit) || candidate.Fraction >= best.Fraction)
					{
						continue;
					}

					candidate.Body = body;
					candidate.OtherBody = otherBody;
					candidate.Shape = shape;
					candidate.OtherShape = otherShape;
					if (DistanceFieldCollision::HasField(shape, otherShape) && !candidate.AngularHit)
						candidate.Position += otherDisplacement * candidate.Fraction;
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
		std::vector<ContinuousCollisionHit>* hits, CollisionFilter* Filter)
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
						if (!CollisionFilter::Allows(Filter, shapeA->GetFilterData(), shapeB->GetFilterData())) continue;
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
						if (!NeedsFieldAngularSweep(shapeA, bodyA, shapeB, bodyB, dt) && relativeDisplacement.SquareLength() > 1.0e-10f)
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
						if (!found || (candidate.Fraction <= 1.0e-5f && !candidate.ConservativeHit) || candidate.Fraction > 1.0f)
						{
							continue;
						}

						candidate.Body = bodyA;
						candidate.OtherBody = bodyB;
						candidate.Shape = shapeA;
						candidate.OtherShape = shapeB;
						if (DistanceFieldCollision::HasField(shapeA, shapeB) && !candidate.AngularHit)
							candidate.Position += (reversed ? displacementA : displacementB) * candidate.Fraction;
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

	int ContinuousCollisionDetection::Resolve(std::vector<RigidBodyDynamic*>& dynamicBodies, const std::vector<RigidBody*>& bodies, float dt, CollisionFilter* Filter)
	{
		for (RigidBodyDynamic* body : dynamicBodies)
		{
			body->CCDPoseResolved = false;
			body->CCDMotionFraction = 1.0f;
			body->HadCCDContact = false;
			body->CCDPreSolveVelocity = body->V;
			body->CCDPreSolveAngularVelocity = body->W;
		}

		int hitCount = 0;
		for (RigidBodyDynamic* body : dynamicBodies)
		{
			ContinuousCollisionHit hit;
			if (!FindEarliestHit(body, bodies, dt, &hit, Filter))
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
			const bool FieldContact = DistanceFieldCollision::HasField(hit.Shape, hit.OtherShape);
			Vector3 contactPosition = FieldContact ? hit.Position : GetSupportPoint(hit.Shape, body, impactTime, -normal);
			Vector3 relativePointVelocity = GetPointVelocity(body, contactPosition, impactTime)
				- GetPointVelocity(hit.OtherBody, contactPosition, impactTime);
			if (relativePointVelocity.Dot(normal) > 0.0f)
			{
				normal = -normal;
				if (!FieldContact) contactPosition = GetSupportPoint(hit.Shape, body, impactTime, -normal);
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

	bool CCDContinuationSettings::IsValid() const
	{
		return MaxPasses > 0 && ImpactIterations > 0
			&& std::isfinite(SimultaneousTolerance) && SimultaneousTolerance > 0 && SimultaneousTolerance < 1
			&& std::isfinite(VelocityTolerance) && VelocityTolerance >= 0
			&& std::isfinite(ContactTolerance) && ContactTolerance > 0
			&& std::isfinite(RestitutionThreshold) && RestitutionThreshold >= 0;
	}

	/*
	CCD continuation uses a shared event clock: sweep, advance every awake body to
	the earliest impact, solve its simultaneous contacts and joints, then resweep
	the remaining interval. A discrete body receiving an impact joins the swept
	set, as do joint-connected bodies. This avoids stale trajectories in chains.
	The implementation uses the existing Riemann shape queries.

	MaxPasses bounds work per impacted body. Exhausted bodies and their joint
	connections discard remaining motion; unrelated bodies still advance. Clamped
	bodies act as immovable targets for the rest of this frame, then recover their
	post-impact velocity for the next frame. A conservative SDF result without a
	certified witness also clamps motion. ResweepCount includes the initial search.
	No artificial minimum time step is taken through a touching obstacle.

	Impacts use coupled projected normal impulses and the material restitution
	maximum. Restitution targets are captured once per event, never per iteration.
	Joint SolveVelocity receives the original frame duration so drive force caps
	and reaction impulse accounting remain per-frame. CCD does not add friction;
	the ordinary contact solvers own friction and overlap recovery.

	Temporary poses describe the event clock. The final pose is cached and the
	input pose restored for the normal SI/LCP/PBD/XPBD integration phase. Physical
	post-impact velocity must not be reconstructed from this piecewise trajectory.
	This route uses exponential rotation for all swept interval predictions.
	Non-SDF angular queries retain the existing bounded sampling approximation;
	this scheduling algorithm does not turn them into certified angular sweeps.
	Near an SDF surface, separating point velocity alone cannot certify the rest
	of a nonconvex or rotating path. Unresolved starts conservatively drop time.
	Trigger callbacks are deferred until the actual segmented path is known;
	callback velocity changes therefore affect the following frame's motion.
	*/
	namespace
	{
		struct CCDMotionState
		{
			RigidBodyDynamic* Body;
			Vector3 Position, Velocity, AngularVelocity;
			Vector3 ResidualVelocity, ResidualAngularVelocity;
			Quaternion Rotation;
			MorionType MotionType;
			int Passes = 0;
			int LastImpactPass = 0;
			bool Swept = false;
			bool Stopped = false;
		};

		struct CCDImpactRow
		{
			ContinuousCollisionHit Hit;
			Vector3 Point, Normal, ArmA, ArmB;
			Matrix3 InertiaA, InertiaB;
			float InverseMassA = 0, InverseMassB = 0;
			float Weight = 0, TargetSpeed = 0, Impulse = 0;
		};

		class CCDContinuation
		{
		public:
			CCDContinuation(const std::vector<RigidBodyDynamic*>& DynamicBodies,
				const std::vector<RigidBody*>& Bodies, float Dt, const CCDContinuationSettings& Settings,
				const std::vector<PositionConstraint*>& Joints, std::vector<ContinuousCollisionHit>* Triggers, CollisionFilter* Filter)
				: m_Bodies(Bodies), m_Dt(Dt), m_Settings(Settings), m_Joints(Joints), m_Triggers(Triggers), m_Filter(Filter)
			{
				m_States.reserve(DynamicBodies.size());
				m_Indices.reserve(DynamicBodies.size());
				for (auto* Body : DynamicBodies)
				{
					CCDMotionState State;
					State.Body = Body; State.Position = Body->X; State.Rotation = Body->Q;
					State.Velocity = Body->V; State.AngularVelocity = Body->W; State.MotionType = Body->mMotionType;
					State.Swept = Body->mMotionType == MorionType::Continuous;
					m_Indices[Body] = m_States.size(); m_States.push_back(State);
				}
				for (auto* Body : Bodies) if (Body)
					for (auto* Shape : Body->Geometries())
						if (Shape && Shape->IsTriggerEnabled() && Shape->IsTriggerCCDEnabled()) m_HasTriggerCCD = true;
			}

			~CCDContinuation()
			{
				if (m_Active) RestoreMotion(false);
			}

			CCDContinuationStats Run()
			{
				// Allocate all snapshots before borrowing poses and motion modes. Query
				// callbacks may throw; the destructor unwinds this temporary event clock
				// while retaining physical impulses, as required by failed world steps.
				m_Active = true;
				for (auto& State : m_States)
				{
					auto* Body = State.Body;
					Body->CCDPoseResolved = false; Body->HadCCDContact = false; Body->CCDMotionFraction = 1;
					Body->CCDPreSolveVelocity = Body->V; Body->CCDPreSolveAngularVelocity = Body->W;
					Body->mMotionType = MorionType::Continuous;
					if (Body->Sleeping || Body->Freezing) { Body->V.SetZero(); Body->W.SetZero(); }
				}
				if (m_Triggers) m_Triggers->clear();
				float Time = 0;
				while (Time < m_Dt)
				{
					PropagateJoints(false, Time);
					const float Remaining = m_Dt - Time;
					std::vector<ContinuousCollisionHit> Hits;
					FindHits(Remaining, Hits); ++m_Stats.ResweepCount;
					if (Hits.empty()) { Advance(Time, Remaining); Time = m_Dt; break; }
					float Fraction = 1;
					for (const auto& Hit : Hits) Fraction = std::min(Fraction, Hit.Fraction);
					const float Interval = Remaining * Fraction;
					Advance(Time, Interval); Time += Interval;
					Hits.erase(std::remove_if(Hits.begin(), Hits.end(), [&](const ContinuousCollisionHit& Hit)
						{ return Hit.Fraction > Fraction + m_Settings.SimultaneousTolerance; }), Hits.end());
					std::vector<CCDImpactRow> Rows;
					for (const auto& Hit : Hits)
					{
						MarkImpact(Hit.Body, Time); MarkImpact(Hit.OtherBody, Time);
						++m_Stats.ImpactCount;
						if (Hit.ConservativeHit)
						{
							Stop(StateOf(Hit.Body), Time); Stop(StateOf(Hit.OtherBody), Time);
						}
					}
					PropagateJoints(false, Time);
					PropagateJoints(true, Time);
					for (const auto& Hit : Hits) if (!Hit.ConservativeHit) Rows.push_back(MakeRow(Hit));
					SolveImpacts(Rows);
					for (auto& State : m_States)
						if (Time < m_Dt && State.Passes >= m_Settings.MaxPasses) Stop(&State, Time);
					PropagateJoints(true, Time);
				}
				RestoreMotion(true);
				return m_Stats;
			}

		private:
			void RestoreMotion(bool Commit)
			{
				for (auto& State : m_States)
				{
					auto* Body = State.Body;
					if (Commit) { Body->CCDTargetPosition = Body->X; Body->CCDTargetRotation = Body->Q; }
					else { Body->HadCCDContact = false; Body->CCDMotionFraction = 1; }
					Body->CCDPoseResolved = Commit && !Body->Sleeping && !Body->Freezing;
					Body->X = State.Position; Body->Q = State.Rotation; Body->mMotionType = State.MotionType;
					if (State.Stopped) { Body->V = State.ResidualVelocity; Body->W = State.ResidualAngularVelocity; }
					else if (Body->Sleeping || Body->Freezing) { Body->V = State.Velocity; Body->W = State.AngularVelocity; }
					Body->UpdateGeometries();
				}
				if (!Commit && m_Triggers) m_Triggers->clear();
				m_Active = false;
			}
			CCDMotionState* StateOf(RigidBody* Body)
			{
				auto Found = m_Indices.find(Body);
				return Found == m_Indices.end() ? nullptr : &m_States[Found->second];
			}
			bool Movable(CCDMotionState* State) const
			{
				return State && !State->Stopped && !State->Body->Freezing;
			}
			void MarkImpact(RigidBody* Body, float Time)
			{
				auto* State = StateOf(Body); if (!Movable(State)) return;
				State->Swept = true;
				if (State->LastImpactPass != m_Stats.ResweepCount) ++State->Passes;
				State->LastImpactPass = m_Stats.ResweepCount;
				State->Body->HadCCDContact = true;
				State->Body->CCDMotionFraction = std::min(State->Body->CCDMotionFraction, Time / m_Dt);
			}
			void Stop(CCDMotionState* State, float Time)
			{
				if (!Movable(State)) return;
				State->Stopped = true;
				State->ResidualVelocity = State->Body->V; State->ResidualAngularVelocity = State->Body->W;
				State->Body->V.SetZero(); State->Body->W.SetZero();
				++m_Stats.ClampedBodyCount;
				m_Stats.MaximumDroppedTime = std::max(m_Stats.MaximumDroppedTime, m_Dt - Time);
			}
			void PropagateJoints(bool Stopping, float Time)
			{
				bool Changed = true;
				while (Changed)
				{
					Changed = false;
					for (auto* Joint : m_Joints)
					{
						if (!Joint || !Joint->IsEnabled()) continue;
						auto* A = StateOf(Joint->GetBodyA()); auto* B = StateOf(Joint->GetBodyB());
						if (!A || !B || A->Body->Freezing || B->Body->Freezing) continue;
						if (Stopping)
						{
							if (A->Stopped != B->Stopped) { Stop(A->Stopped ? B : A, Time); Changed = true; }
						}
						else if (A->Swept != B->Swept) { A->Swept = B->Swept = true; Changed = true; }
					}
				}
			}
			void Advance(float Time, float Interval)
			{
				if (Interval <= 0) return;
				if (m_Triggers && m_HasTriggerCCD)
				{
					std::vector<ContinuousCollisionHit> Hits;
					ContinuousCollisionDetection::FindTriggerHits(m_Bodies, Interval, &Hits, m_Filter);
					for (auto Hit : Hits)
					{
						const bool Seen = std::any_of(m_Triggers->begin(), m_Triggers->end(), [&](const ContinuousCollisionHit& Old)
							{ return Old.Shape == Hit.Shape && Old.OtherShape == Hit.OtherShape; });
						if (!Seen) { Hit.Fraction = (Time + Hit.Fraction * Interval) / m_Dt; m_Triggers->push_back(Hit); }
					}
				}
				for (auto& State : m_States)
				{
					if (!Movable(&State) || State.Body->Sleeping) continue;
					State.Body->Q = PredictRotation(State.Body, Interval);
					State.Body->X += State.Body->V * Interval;
					State.Body->UpdateGeometries();
				}
			}
			bool MeshTouchingNormal(RigidBody* A, Geometry* ShapeA, RigidBody* B, Geometry* ShapeB, Vector3& Normal)
			{
				GeometryWorldState StateA, StateB;
				StateA.Geom = ShapeA; StateA.Body = A; StateA.ShapeToWorld = PredictGeometryTransform(A, ShapeA, 0);
				StateB.Geom = ShapeB; StateB.Body = B; StateB.ShapeToWorld = PredictGeometryTransform(B, ShapeB, 0);
				std::vector<ContactManifold> Patches;
				m_MeshContacts.Generate(0, 1, StateA, StateB, Patches, m_Settings.ContactTolerance);
				float MinimumSpeed = FLT_MAX;
				for (const auto& Patch : Patches)
				for (int I = 0; I < Patch.NumContactPointCount; ++I)
				{
					const auto& Contact = Patch.ContactPoints[I];
					const float Speed = (GetPointVelocity(A, Contact.PositionWorldA, 0)
						- GetPointVelocity(B, Contact.PositionWorldA, 0)).Dot(-Contact.Normal);
					if (Speed < MinimumSpeed) { MinimumSpeed = Speed; Normal = -Contact.Normal; }
				}
				return MinimumSpeed != FLT_MAX;
			}
			bool QueryPair(RigidBody* A, Geometry* ShapeA, RigidBody* B, Geometry* ShapeB, float Remaining, ContinuousCollisionHit& Hit)
			{
				const Vector3 Motion = (A->V - B->V) * Remaining;
				bool Found = !NeedsFieldAngularSweep(ShapeA, A, ShapeB, B, Remaining)
					&& !Motion.IsZero() && SweepPair(ShapeA, A, ShapeB, B, Motion, &Hit);
				const bool Field = DistanceFieldCollision::HasField(ShapeA, ShapeB);
				if (!Found && Field)
				{
					// SweepBounded delegates initial overlap to discrete collision. A
					// resweep may instead have just transferred velocity into that contact.
					DistanceFieldPairDistance Distance;
					if (DistanceFieldCollision::Distance(ShapeA, PredictGeometryTransform(A, ShapeA, 0),
						ShapeB, PredictGeometryTransform(B, ShapeB, 0), Distance, m_Settings.ContactTolerance / 2)
						&& Distance.WitnessValid && Distance.UpperBound <= m_Settings.ContactTolerance)
					{
						Hit.Fraction = 0; Hit.Position = Distance.PointA; Hit.Normal = Distance.Normal; Found = true;
					}
				}
				if (Found && Field) Hit.Position += B->V * (Remaining * Hit.Fraction);
				if (Found && Hit.Fraction <= m_Settings.SimultaneousTolerance && !Field)
				{
					// A touching convex sweep's default normal is -motion, which would
					// incorrectly stop tangential sliding. Recover the geometric normal.
					if (IsConvexShape(ShapeB->GetShapeType()))
					{
						TransformedMinkowskiSum<Geometry, Geometry> Difference(ShapeB, GetBodyTransform(B), ShapeA, GetBodyTransform(A));
						GJKIntersection Distance;
						if (Distance.Solve(&Difference) == GJK_status::NotIntersect && !Distance.closest.IsZero())
							Hit.Normal = -Distance.closest.SafeUnit();
						else Hit.Normal = EstimateAngularContactNormal(ShapeA, A, PredictGeometryTransform(A, ShapeA, 0),
							ShapeB, B, PredictGeometryTransform(B, ShapeB, 0), 0);
					}
					else if (ShapeB->GetShapeType() == PrimitiveType::TRIANGLE_MESH || ShapeB->GetShapeType() == PrimitiveType::HEIGHTFIELD)
					{
						if (!MeshTouchingNormal(A, ShapeA, B, ShapeB, Hit.Normal)) Hit.ConservativeHit = true;
					}
				}
				// Reject separating linear contacts before comparing times. Otherwise
				// a harmless contact at t=0 can hide a later angular re-impact.
				Found = Found && PrepareHit(A, ShapeA, B, ShapeB, Remaining, Hit);
				ContinuousCollisionHit Angular;
				if (SweepAngularPair(ShapeA, A, ShapeB, B, Remaining, &Angular, true)
					&& PrepareHit(A, ShapeA, B, ShapeB, Remaining, Angular)
					&& (!Found || Angular.Fraction < Hit.Fraction)) { Hit = Angular; Found = true; }
				return Found;
			}
			bool PrepareHit(RigidBody* A, Geometry* ShapeA, RigidBody* B, Geometry* ShapeB, float Remaining, ContinuousCollisionHit& Hit)
			{
				if (Hit.Fraction < 0 || Hit.Fraction > 1) return false;
				Hit.Body = A; Hit.OtherBody = B; Hit.Shape = ShapeA; Hit.OtherShape = ShapeB;
				const float Time = Remaining * Hit.Fraction;
				if (ShapeB->GetShapeType() == PrimitiveType::PLANE)
					Hit.Normal = PredictGeometryTransform(B, ShapeB, Time).LocalToWorldDirection(Vector3::UnitY());
				Hit.Normal = Hit.Normal.SafeUnit();
				if (!DistanceFieldCollision::HasField(ShapeA, ShapeB)) Hit.Position = GetSupportPoint(ShapeA, A, Time, -Hit.Normal);
				const float Speed = (GetPointVelocity(A, Hit.Position, Time) - GetPointVelocity(B, Hit.Position, Time)).Dot(Hit.Normal);
				if (DistanceFieldCollision::HasField(ShapeA, ShapeB) && Hit.Fraction <= m_Settings.SimultaneousTolerance
					&& Speed >= -m_Settings.VelocityTolerance)
				{
					// The local field gradient cannot certify that a separating body
					// misses another lobe later. Pure translation away from a plane is
					// safe; otherwise stop when the field query cannot advance from t=0.
					const bool Rotating = (A->W.Length() + B->W.Length()) * Remaining > MinimumAngularSweepTravel;
					const bool Moving = !(A->V - B->V).IsZero() || Rotating;
					if (Moving && (ShapeB->GetShapeType() != PrimitiveType::PLANE || Rotating)) Hit.ConservativeHit = true;
				}
				return Hit.ConservativeHit || Speed < -m_Settings.VelocityTolerance;
			}
			void FindHits(float Remaining, std::vector<ContinuousCollisionHit>& Hits)
			{
				// Enumerate only swept participants against the world. With C CCD
				// bodies among N bodies this is O(C*N), before swept bounds rejection.
				for (size_t I = 0; I < m_States.size(); ++I)
				{
					auto* StateA = &m_States[I]; if (!StateA->Swept) continue;
					auto* A = StateA->Body;
					for (auto* B : m_Bodies)
					{
						if (!B || A == B || !A->CanCollideWith(B)) continue;
						auto* StateB = StateOf(B);
						if (StateB && StateB->Swept && m_Indices[B] < I) continue;
						if ((!Movable(StateA) || StateA->Body->Sleeping) && (!Movable(StateB) || StateB->Body->Sleeping)) continue;
						for (auto* ShapeA : A->Geometries())
						for (auto* ShapeB : B->Geometries())
						{
							if (!ShapeA || !ShapeB || !ShapeA->IsSimulationEnabled() || !ShapeB->IsSimulationEnabled()
								|| ShapeA->IsTriggerEnabled() || ShapeB->IsTriggerEnabled()
								|| !CollisionFilter::Allows(m_Filter, ShapeA->GetFilterData(), ShapeB->GetFilterData())) continue;
							if (ShapeA->GetShapeType() != PrimitiveType::PLANE && ShapeB->GetShapeType() != PrimitiveType::PLANE
								&& !FieldMotionBounds(A, ShapeA, Remaining).Intersect(FieldMotionBounds(B, ShapeB, Remaining))) continue;
							ContinuousCollisionHit Hit;
							if (StateA && ContinuousCollisionDetection::IsSupportedMovingShape(ShapeA))
							{
								if (QueryPair(A, ShapeA, B, ShapeB, Remaining, Hit)) Hits.push_back(Hit);
							}
							else if (StateB && ContinuousCollisionDetection::IsSupportedMovingShape(ShapeB))
							{
								if (QueryPair(B, ShapeB, A, ShapeA, Remaining, Hit)) Hits.push_back(Hit);
							}
						}
					}
				}
			}
			CCDImpactRow MakeRow(const ContinuousCollisionHit& Hit)
			{
				CCDImpactRow Row; Row.Hit = Hit; Row.Point = Hit.Position; Row.Normal = Hit.Normal;
				Row.ArmA = Row.Point - Hit.Body->X; Row.ArmB = Row.Point - Hit.OtherBody->X;
				auto* A = StateOf(Hit.Body); auto* B = StateOf(Hit.OtherBody);
				Row.InverseMassA = Movable(A) ? A->Body->InvMass : 0;
				Row.InverseMassB = Movable(B) ? B->Body->InvMass : 0;
				Row.InertiaA = Movable(A) ? GetInverseInertiaAtTime(A->Body, 0) : Matrix3::Zero();
				Row.InertiaB = Movable(B) ? GetInverseInertiaAtTime(B->Body, 0) : Matrix3::Zero();
				Row.Weight = Row.InverseMassA + Row.InverseMassB
					+ std::max(0.0f, Row.Normal.Dot((Row.InertiaA * Row.ArmA.Cross(Row.Normal)).Cross(Row.ArmA)))
					+ std::max(0.0f, Row.Normal.Dot((Row.InertiaB * Row.ArmB.Cross(Row.Normal)).Cross(Row.ArmB)));
				const float Speed = (GetPointVelocity(Hit.Body, Row.Point, 0) - GetPointVelocity(Hit.OtherBody, Row.Point, 0)).Dot(Row.Normal);
				if (Speed < -m_Settings.RestitutionThreshold)
					Row.TargetSpeed = -Speed * std::max(Hit.Body->GetRestitution(), Hit.OtherBody->GetRestitution());
				return Row;
			}
			void SolveImpacts(std::vector<CCDImpactRow>& Rows)
			{
				for (int Iteration = 0; Iteration < m_Settings.ImpactIterations; ++Iteration)
				{
					for (auto* Joint : m_Joints)
					{
						if (!Joint || !Joint->IsEnabled()) continue;
						auto* A = StateOf(Joint->GetBodyA()); auto* B = StateOf(Joint->GetBodyB());
						if ((A && A->Stopped) || (B && B->Stopped)) continue;
						if ((A && A->Swept) || (B && B->Swept)) Joint->SolveVelocity(m_Dt);
					}
					for (auto& Row : Rows)
					{
						if (Row.Weight <= kMinimumInvMass) continue;
						const float Speed = (GetPointVelocity(Row.Hit.Body, Row.Point, 0)
							- GetPointVelocity(Row.Hit.OtherBody, Row.Point, 0)).Dot(Row.Normal);
						const float NewImpulse = std::max(0.0f, Row.Impulse + (Row.TargetSpeed - Speed) / Row.Weight);
						const Vector3 Impulse = Row.Normal * (NewImpulse - Row.Impulse); Row.Impulse = NewImpulse;
						auto* A = StateOf(Row.Hit.Body); auto* B = StateOf(Row.Hit.OtherBody);
						if (Movable(A)) ApplyContactImpulse(A->Body, Impulse, Row.ArmA, Row.InertiaA);
						if (Movable(B)) ApplyContactImpulse(B->Body, -Impulse, Row.ArmB, Row.InertiaB);
					}
				}
			}

			const std::vector<RigidBody*>& m_Bodies;
			float m_Dt;
			const CCDContinuationSettings& m_Settings;
			const std::vector<PositionConstraint*>& m_Joints;
			std::vector<ContinuousCollisionHit>* m_Triggers;
			CollisionFilter* m_Filter;
			bool m_HasTriggerCCD = false;
			bool m_Active = false;
			std::vector<CCDMotionState> m_States;
			std::unordered_map<RigidBody*, size_t> m_Indices;
			CCDContinuationStats m_Stats;
			MeshContactGenerator m_MeshContacts;
		};
	}

	CCDContinuationStats ContinuousCollisionDetection::ResolveRemainingTime(std::vector<RigidBodyDynamic*>& DynamicBodies,
		const std::vector<RigidBody*>& Bodies, float Dt, const CCDContinuationSettings& Settings,
		const std::vector<PositionConstraint*>& Joints, std::vector<ContinuousCollisionHit>* TriggerHits, CollisionFilter* Filter)
	{
		if (!Settings.IsValid() || !std::isfinite(Dt) || Dt <= 0)
			throw std::invalid_argument("Invalid CCD continuation settings or time step");
		return CCDContinuation(DynamicBodies, Bodies, Dt, Settings, Joints, TriggerHits, Filter).Run();
	}
}
