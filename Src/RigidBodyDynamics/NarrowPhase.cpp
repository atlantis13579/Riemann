
#include "NarrowPhase.h"
#include "CollidingContact.h"
#include "RigidBody.h"
#include "../Core/BatchList.h"
#include "../Geometry/Polygon3.h"
#include "../Collision/GeometryObject.h"
#include "../Collision/GeometryDifference.h"
#include "../Collision/EPAPenetration.h"
#include "../CollisionPrimitive/GJK.h"
#include "../CollisionPrimitive/MinkowskiSum.h"

namespace Riemann
{
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

	static void ConstructManifols(int indexA, int indexB, const GeometryWorldState& StateA, const GeometryWorldState& StateB, EPAPenetration& epa, ContactManifold* manifold)
	{
		manifold->Reset();

		// http://allenchou.net/2013/12/game-physics-contact-generation-epa/
		Vector3 w0 = Vector3::Zero();
		for (int i = 0; i < epa.result.dimension; ++i)
		{
			Vector3 pi = GetGeometrySupport(StateA, epa.result.v[i].dir) * epa.result.w[i];
			w0 = w0 + pi;
		}

		const bool UseContactFace = true;
		SupportFace ContactFace;
		if (UseContactFace)
		{
			bool succ = ConstructSupportFace(StateA, StateB, epa.penetration_normal, ContactFace);
			if (!succ)
			{
				succ = ConstructSupportFace(StateB, StateA, -epa.penetration_normal, ContactFace);
			}
		}

		for (int i = -1; i < ContactFace.size(); ++i)
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
			if (contact.Normal.x >= 0.57735f)
			{
				contact.Tangent.x = contact.Normal.y;
				contact.Tangent.y = -contact.Normal.x;
				contact.Tangent.z = 0;
			}
			else
			{
				contact.Tangent.x = 0;
				contact.Tangent.y = contact.Normal.z;
				contact.Tangent.z = -contact.Normal.y;
			}
			contact.Binormal = CrossProduct(contact.Normal, contact.Tangent);

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
				if (PenetrationTest(state1, state2, epa))
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
