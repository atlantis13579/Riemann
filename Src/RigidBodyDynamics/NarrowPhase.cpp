
#include "NarrowPhase.h"
#include "CollidingContact.h"
#include "../Core/BatchList.h"
#include "../Geometry/Polygon3.h"
#include "../Collision/GeometryObject.h"
#include "../Collision/GeometryDifference.h"
#include "../Collision/EPAPenetration.h"
#include "../CollisionPrimitive/GJK.h"

namespace Riemann
{
	static bool ConstructSupportFace(Geometry* GeomA, Geometry* GeomB, const Vector3& penetration_normal, SupportFace& Face)
	{
		SupportFace FaceA, FaceB;
		GeomA->GetSupportFace_WorldSpace(-penetration_normal, FaceA);
		GeomB->GetSupportFace_WorldSpace(penetration_normal, FaceB);

		const float mSpeculativeContactDistance = 0.02f;
		bool succ = ClipPolygonAgainPolygon3D(FaceA.data(), FaceA.size(), FaceB.data(), FaceB.size(), penetration_normal, mSpeculativeContactDistance, Face.data(), Face.GetSizeData(), nullptr, nullptr);
		return succ;
	}


	static bool PenetrationTest(Geometry* Geom1, Geometry* Geom2, EPAPenetration& epa)
	{
		GeometryDifference shape(Geom1, Geom2);
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

	static void ConstructManifols(int indexA, int indexB, Geometry* GeomA, Geometry* GeomB, EPAPenetration& epa, ContactManifold* manifold)
	{
		manifold->Reset();

		// http://allenchou.net/2013/12/game-physics-contact-generation-epa/
		Vector3 w0 = Vector3::Zero();
		for (int i = 0; i < epa.result.dimension; ++i)
		{
			Vector3 pi = GeomA->GetSupport_WorldSpace(epa.result.v[i].dir) * epa.result.w[i];
			w0 = w0 + pi;
		}

		const bool UseContactFace = true;
		SupportFace ContactFace;
		if (UseContactFace)
		{
			bool succ = ConstructSupportFace(GeomA, GeomB, epa.penetration_normal, ContactFace);
			if (!succ)
			{
				succ = ConstructSupportFace(GeomB, GeomA, -epa.penetration_normal, ContactFace);
			}
		}

		// const Matrix4& invWorldA = GeomA->GetInverseWorldMatrix();
		for (int i = -1; i < ContactFace.size(); ++i)
		{
			Vector3 pa = i == -1 ? w0 : ContactFace[i];
			Contact contact;
			contact.PositionLocalA = pa - GeomA->GetWorldPosition();
			Vector3 pb = pa - epa.penetration_normal * epa.penetration_depth;
			contact.PositionLocalB = pb - GeomB->GetWorldPosition();
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

		virtual void CollisionDetection(const std::vector<Geometry*>& geoms,
			const std::vector<OverlapPair>& overlaps,
			std::vector<ContactManifold*>* manifolds) override final
		{
			m_ManifoldPool.clear();

			manifolds->clear();
			for (size_t i = 0; i < overlaps.size(); ++i)
			{
				Geometry* geom1 = geoms[overlaps[i].index1];
				Geometry* geom2 = geoms[overlaps[i].index2];

				EPAPenetration epa;
				if (PenetrationTest(geom1, geom2, epa))
				{
					ContactManifold* manifold = m_ManifoldPool.allocate();
					ConstructManifols(overlaps[i].index1, overlaps[i].index2, geom1, geom2, epa, manifold);
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

		virtual void CollisionDetection(const std::vector<Geometry*>& geoms,
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

		virtual void CollisionDetection(const std::vector<Geometry*>& geoms,
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

		virtual void CollisionDetection(const std::vector<Geometry*>& geoms,
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