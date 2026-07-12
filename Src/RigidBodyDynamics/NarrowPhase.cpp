
#include "NarrowPhase.h"
#include <algorithm>
#include <cmath>
#include "CollidingContact.h"
#include "RigidBody.h"
#include "../Core/BatchList.h"
#include "../Geometry/Polygon3.h"
#include "../Collision/GeometryObject.h"
#include "../Collision/GeometryDifference.h"
#include "../Collision/EPAPenetration.h"
#include "../CollisionPrimitive/AxisAlignedBox3.h"
#include "../CollisionPrimitive/GJK.h"
#include "../CollisionPrimitive/MinkowskiSum.h"
#include "../CollisionPrimitive/OrientedBox3.h"

namespace Riemann
{
	static bool IsConcaveGeometry(const Geometry* GeometryObject)
	{
		if (GeometryObject == nullptr)
		{
			return false;
		}
		const PrimitiveType Type = GeometryObject->GetShapeType();
		return Type == PrimitiveType::TRIANGLE_MESH || Type == PrimitiveType::HEIGHTFIELD;
	}

	static Vector3 GetGeometrySupport(const GeometryWorldState& State, const Vector3& Direction)
	{
		if (State.Geom == nullptr)
		{
			return Vector3::Zero();
		}

		if (State.Body == nullptr)
		{
			return State.Geom->GetSupport(Direction);
		}

		const Vector3 LocalDirection = State.BodyToWorld.WorldToLocalDirection(Direction);
		const Vector3 LocalSupport = State.Geom->GetSupport(LocalDirection);
		return State.BodyToWorld.LocalToWorld(LocalSupport);
	}

	static void GetGeometrySupportFace(const GeometryWorldState& State, const Vector3& Direction, SupportFace& Face)
	{
		if (State.Geom == nullptr)
		{
			return;
		}

		if (State.Body == nullptr)
		{
			State.Geom->GetSupportFace(Direction, Face);
			return;
		}

		const Vector3 LocalDirection = State.BodyToWorld.WorldToLocalDirection(Direction);
		State.Geom->GetSupportFace(LocalDirection, Face);
		for (int i = 0; i < Face.size(); ++i)
		{
			Face[i] = State.BodyToWorld.LocalToWorld(Face[i]);
		}
	}

	static Vector3 GetContactRelativePosition(const GeometryWorldState& State, const Vector3& Position)
	{
		if (State.Body)
		{
			return Position - State.Body->X;
		}
		return State.Geom ? Position - State.Geom->GetPosition() : Position;
	}

	static bool ConstructSupportFace(const GeometryWorldState& StateA, const GeometryWorldState& StateB, const Vector3& penetration_normal, SupportFace& Face)
	{
		SupportFace FaceA, FaceB;
		GetGeometrySupportFace(StateA, -penetration_normal, FaceA);
		GetGeometrySupportFace(StateB, penetration_normal, FaceB);

		const float mSpeculativeContactDistance = 0.02f;
		bool succ = ClipPolygonAgainPolygon3D(FaceA.data(), FaceA.size(), FaceB.data(), FaceB.size(), penetration_normal, mSpeculativeContactDistance, Face.data(), Face.GetSizeData(), nullptr, nullptr);
		return succ;
	}


	static bool PenetrationTest(const GeometryWorldState& State1, const GeometryWorldState& State2, EPAPenetration& epa)
	{
		if (IsConcaveGeometry(State1.Geom) || IsConcaveGeometry(State2.Geom))
		{
			return false;
		}
		if (State1.Geom->GetShapeType() == PrimitiveType::BOX
			&& State2.Geom->GetShapeType() == PrimitiveType::BOX)
		{
			const AxisAlignedBox3* Box1 = State1.Geom->GetShapeObj<AxisAlignedBox3>();
			const AxisAlignedBox3* Box2 = State2.Geom->GetShapeObj<AxisAlignedBox3>();
			const OrientedBox3 OBB1(State1.ShapeToWorld.LocalToWorld(Box1->GetCenter()),
				Box1->GetExtent(), State1.ShapeToWorld.quat.ToRotationMatrix3());
			const OrientedBox3 OBB2(State2.ShapeToWorld.LocalToWorld(Box2->GetCenter()),
				Box2->GetExtent(), State2.ShapeToWorld.quat.ToRotationMatrix3());
			if (!OBB1.PenetrateOBB(OBB2, &epa.penetration_normal, &epa.penetration_depth))
			{
				return false;
			}
			epa.penetration_normal.SafeNormalize();
			epa.penetration_depth = std::max(0.0f, epa.penetration_depth);
			epa.status = EPA_status::Valid;
			return true;
		}
		TransformedMinkowskiSum<Geometry, Geometry> shape(
			State1.Geom, State1.BodyToWorld,
			State2.Geom, State2.BodyToWorld);
		GJKIntersection gjk;
		GJK_status gjk_status = gjk.Solve(&shape);
		if (gjk_status != GJK_status::Intersect)
		{
			return false;
		}

		EPA_status epa_result = epa.Solve(gjk.result);
		if (epa_result == EPA_status::Failed || epa_result == EPA_status::FallBack)
		{
			return false;
		}

		return true;
	}

	static void BuildContactBasis(Contact* contact)
	{
		if (std::fabs(contact->Normal.x) >= 0.57735f)
		{
			contact->Tangent = Vector3(contact->Normal.y, -contact->Normal.x, 0.0f);
		}
		else
		{
			contact->Tangent = Vector3(0.0f, contact->Normal.z, -contact->Normal.y);
		}
		contact->Tangent.SafeNormalize();
		contact->Binormal = CrossProduct(contact->Normal, contact->Tangent);
	}

	static bool ConstructConcaveManifold(int indexA, int indexB, const GeometryWorldState& StateA, const GeometryWorldState& StateB, ContactManifold* manifold)
	{
		const bool ConcaveIsA = IsConcaveGeometry(StateA.Geom);
		const bool ConcaveIsB = IsConcaveGeometry(StateB.Geom);
		if (ConcaveIsA == ConcaveIsB)
		{
			return false;
		}
		const GeometryWorldState& ConcaveState = ConcaveIsA ? StateA : StateB;
		const GeometryWorldState& ConvexState = ConcaveIsA ? StateB : StateA;
		manifold->Reset();

		const Vector3 RelativeDisplacement = ConvexState.Displacement - ConcaveState.Displacement;
		const bool CurrentlyIntersecting = ConvexState.Geom->Intersect(
			ConvexState.ShapeToWorld, ConcaveState.Geom, ConcaveState.ShapeToWorld);
		const PrimitiveType ConvexType = ConvexState.Geom->GetShapeType();
		const bool CanEstimatePenetration = ConvexType == PrimitiveType::BOX
			|| ConvexType == PrimitiveType::SPHERE || ConvexType == PrimitiveType::CAPSULE;
		auto EstimatePenetration = [&](const Vector3& WorldNormal, float InitialDepth) -> float
		{
			if (!CanEstimatePenetration)
			{
				return std::max(0.001f, InitialDepth);
			}
			float Lower = 0.0f;
			float Upper = 0.01f;
			Transform SeparatedTransform = ConvexState.ShapeToWorld;
			SeparatedTransform.pos += WorldNormal * Upper;
			while (Upper < 0.5f && ConvexState.Geom->Intersect(SeparatedTransform, ConcaveState.Geom, ConcaveState.ShapeToWorld))
			{
				Lower = Upper;
				Upper *= 2.0f;
				SeparatedTransform.pos = ConvexState.ShapeToWorld.pos + WorldNormal * Upper;
			}
			for (int Iteration = 0; Iteration < 8; ++Iteration)
			{
				const float Middle = (Lower + Upper) * 0.5f;
				SeparatedTransform.pos = ConvexState.ShapeToWorld.pos + WorldNormal * Middle;
				if (ConvexState.Geom->Intersect(SeparatedTransform, ConcaveState.Geom, ConcaveState.ShapeToWorld))
				{
					Lower = Middle;
				}
				else
				{
					Upper = Middle;
				}
			}
			return std::min(0.2f, std::max(InitialDepth, Upper));
		};

		struct ConcaveContactCandidate
		{
			Vector3 Position = Vector3::Zero();
			Vector3 Normal = Vector3::Zero();
			float Depth = 1.0e30f;
			bool Valid = false;
		};
		ConcaveContactCandidate BestCandidate;
		auto TrySweepCandidate = [&](const Transform& StartTransform, const Vector3& WorldDirection)
		{
			Vector3 LocalPosition;
			Vector3 LocalNormal;
			float HitTime = 0.0f;
			const Vector3 LocalDirection = StartTransform.WorldToLocalDirection(WorldDirection);
			if (!ConvexState.Geom->Sweep(StartTransform, LocalDirection, ConcaveState.Geom,
				ConcaveState.ShapeToWorld, &LocalPosition, &LocalNormal, &HitTime)
				|| HitTime < 0.0f || HitTime > 1.0f)
			{
				return;
			}
			// An initial-overlap hit from the previous pose is not a collision when
			// the current pose has already separated from the surface.
			if (!CurrentlyIntersecting && HitTime <= 1.0e-5f)
			{
				return;
			}

			Vector3 WorldNormal = StartTransform.LocalToWorldDirection(LocalNormal).SafeUnit();
			if (WorldNormal.IsZero())
			{
				return;
			}
			if (WorldNormal.Dot(WorldDirection) > 0.0f)
			{
				WorldNormal = -WorldNormal;
			}
			if (ConcaveState.Geom->GetShapeType() == PrimitiveType::HEIGHTFIELD)
			{
				const Vector3 HeightFieldNormal = ConcaveState.ShapeToWorld.LocalToWorldDirection(Vector3::UnitY());
				if (WorldNormal.Dot(HeightFieldNormal) < 0.0f)
				{
					WorldNormal = -WorldNormal;
				}
			}

			const float ImpactDepth = CurrentlyIntersecting ? 0.001f : std::max(0.001f,
				-RelativeDisplacement.Dot(WorldNormal) * std::max(0.0f, 1.0f - HitTime));
			const float PenetrationDepth = EstimatePenetration(WorldNormal, ImpactDepth);
			if (!BestCandidate.Valid || PenetrationDepth < BestCandidate.Depth)
			{
				BestCandidate.Valid = true;
				BestCandidate.Position = StartTransform.LocalToWorld(LocalPosition);
				BestCandidate.Normal = WorldNormal;
				BestCandidate.Depth = PenetrationDepth;
			}
		};

		if (CurrentlyIntersecting)
		{
			const Vector3 ProbeAxes[3] = {
				ConcaveState.ShapeToWorld.LocalToWorldDirection(Vector3::UnitX()),
				ConcaveState.ShapeToWorld.LocalToWorldDirection(Vector3::UnitY()),
				ConcaveState.ShapeToWorld.LocalToWorldDirection(Vector3::UnitZ())
			};
			for (const Vector3& ProbeAxis : ProbeAxes)
			{
				TrySweepCandidate(ConvexState.ShapeToWorld, ProbeAxis * 0.02f);
				TrySweepCandidate(ConvexState.ShapeToWorld, ProbeAxis * -0.02f);
			}
		}
		else if (RelativeDisplacement.SquareLength() > 1.0e-10f)
		{
			Transform StartTransform = ConvexState.ShapeToWorld;
			StartTransform.pos -= RelativeDisplacement;
			TrySweepCandidate(StartTransform, RelativeDisplacement);
		}
		if (!BestCandidate.Valid)
		{
			return false;
		}

		Contact contact;
		contact.PositionWorldA = BestCandidate.Position;
		contact.PositionWorldB = BestCandidate.Position - BestCandidate.Normal * BestCandidate.Depth;
		contact.Normal = BestCandidate.Normal;
		contact.PenetrationDepth = BestCandidate.Depth;
		if (!ConcaveIsA)
		{
			std::swap(contact.PositionWorldA, contact.PositionWorldB);
			contact.Normal = -contact.Normal;
		}
		BuildContactBasis(&contact);
		contact.PositionLocalA = GetContactRelativePosition(StateA, contact.PositionWorldA);
		contact.PositionLocalB = GetContactRelativePosition(StateB, contact.PositionWorldB);
		manifold->AddNewContact(indexA, indexB, contact);
		return true;
	}

	static void ConstructManifols(int indexA, int indexB, const GeometryWorldState& StateA, const GeometryWorldState& StateB, EPAPenetration& epa, ContactManifold* manifold)
	{
		manifold->Reset();

		// http://allenchou.net/2013/12/game-physics-contact-generation-epa/
		Vector3 w0 = Vector3::Zero();
		if (epa.result.dimension == 0)
		{
			w0 = GetGeometrySupport(StateA, epa.penetration_normal);
		}
		else
		{
			for (int i = 0; i < epa.result.dimension; ++i)
			{
				Vector3 pi = GetGeometrySupport(StateA, epa.result.v[i].dir) * epa.result.w[i];
				w0 = w0 + pi;
			}
		}

		const bool UseContactFace = true;
		SupportFace ContactFace;
		bool bHasContactFace = false;
		if (UseContactFace)
		{
			bool succ = ConstructSupportFace(StateA, StateB, epa.penetration_normal, ContactFace);
			if (!succ)
			{
				succ = ConstructSupportFace(StateB, StateA, -epa.penetration_normal, ContactFace);
			}
			bHasContactFace = succ && ContactFace.size() > 0;
		}

		const int FirstContactIndex = bHasContactFace ? 0 : -1;
		for (int i = FirstContactIndex; i < ContactFace.size(); ++i)
		{
			Vector3 pa = i == -1 ? w0 : ContactFace[i];
			Contact contact;
			contact.PositionLocalA = GetContactRelativePosition(StateA, pa);
			Vector3 pb = pa - epa.penetration_normal * epa.penetration_depth;
			contact.PositionLocalB = GetContactRelativePosition(StateB, pb);
			contact.PositionWorldA = pa;
			contact.PositionWorldB = pb;
			contact.Normal = epa.penetration_normal;
			contact.PenetrationDepth = epa.penetration_depth;
			BuildContactBasis(&contact);

			manifold->AddNewContact(indexA, indexB, contact);
		}
	}


	class NarrowPhase_GJKEPA : public NarrowPhase
	{
	public:
		NarrowPhase_GJKEPA()
		{
			m_ManifoldPool.init(1, 256);
		}
		virtual ~NarrowPhase_GJKEPA()
		{

		}

		virtual void CollisionDetection(GeometryWorldStateSpan states,
			const std::vector<OverlapPair>& overlaps,
			std::vector<ContactManifold*>* manifolds) override final
		{
			m_ManifoldPool.clear();

			manifolds->clear();
			for (size_t i = 0; i < overlaps.size(); ++i)
			{
				const GeometryWorldState& state1 = states[overlaps[i].index1];
				const GeometryWorldState& state2 = states[overlaps[i].index2];
				if (state1.Geom == nullptr || state2.Geom == nullptr)
				{
					continue;
				}

				EPAPenetration epa;
				if (IsConcaveGeometry(state1.Geom) || IsConcaveGeometry(state2.Geom))
				{
					ContactManifold* manifold = m_ManifoldPool.allocate();
					if (ConstructConcaveManifold(overlaps[i].index1, overlaps[i].index2, state1, state2, manifold))
					{
						manifolds->push_back(manifold);
					}
				}
				else if (PenetrationTest(state1, state2, epa))
				{
					ContactManifold* manifold = m_ManifoldPool.allocate();
					ConstructManifols(overlaps[i].index1, overlaps[i].index2, state1, state2, epa, manifold);
					manifolds->push_back(manifold);
				}
			}

			return;
		}

	private:
		BatchList<ContactManifold>	m_ManifoldPool;
	};

	class NarrowPhase_PenetrateTable : public NarrowPhase
	{
	public:
		NarrowPhase_PenetrateTable()
		{
			m_ManifoldPool.init(1, 256);
		}
		virtual ~NarrowPhase_PenetrateTable()
		{

		}

		virtual void CollisionDetection(GeometryWorldStateSpan states,
			const std::vector<OverlapPair>& overlaps,
			std::vector<ContactManifold*>* manifolds) override final
		{
			return;
		}

	private:
		BatchList<ContactManifold>	m_ManifoldPool;
	};

	class NarrowPhase_SAT : public NarrowPhase
	{
	public:
		NarrowPhase_SAT()
		{
		}
		virtual ~NarrowPhase_SAT()
		{
		}

		virtual void CollisionDetection(GeometryWorldStateSpan states,
			const std::vector<OverlapPair>& overlaps,
			std::vector<ContactManifold*>* manifolds) override final
		{
			return;
		}
	};

	class NarrowPhase_PersistentContactManifold : public NarrowPhase
	{
	public:
		NarrowPhase_PersistentContactManifold()
		{
		}
		virtual ~NarrowPhase_PersistentContactManifold()
		{
		}

		virtual void CollisionDetection(GeometryWorldStateSpan states,
			const std::vector<OverlapPair>& overlaps,
			std::vector<ContactManifold*>* manifolds) override final
		{
			return;
		}

	private:
		BatchList<ContactManifold>	m_ManifoldPool;
	};

	NarrowPhase* NarrowPhase::Create_GJKEPA()
	{
		return new NarrowPhase_GJKEPA();
	}

	NarrowPhase* NarrowPhase::Create_SAT()
	{
		return new NarrowPhase_SAT();
	}

	NarrowPhase* NarrowPhase::Create_PCM()
	{
		return new NarrowPhase_PersistentContactManifold();
	}
}
