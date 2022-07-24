
#include "NarrowPhase.h"
#include "Contact.h"
#include "../Geometry/Polygon3d.h"
#include "../Collision/GeometryObject.h"
#include "../Collision/GeometryDifference.h"
#include "../Collision/EPAPenetration.h"
#include "../Collision/GJK.h"

class NarrowPhase_GJKEPA : public NarrowPhase
{
public:
	NarrowPhase_GJKEPA()
	{
		mUseContactFace = true;
	}
	virtual ~NarrowPhase_GJKEPA()
	{

	}

	virtual void CollisionDetection(std::vector<OverlapPair>& overlaps, std::vector<ContactManifold>* manifolds) override final
	{
		manifolds->clear();
		for (size_t i = 0; i < overlaps.size(); ++i)
		{
			EPAPenetration epa;
			if (PenetrationTest(overlaps[i].Geom1, overlaps[i].Geom2, epa))
			{
				ConstructManifols(overlaps[i].Geom1, overlaps[i].Geom2, epa, manifolds);
			}
		}

		return;
	}

	bool PenetrationTest(Geometry* Geom1, Geometry* Geom2, EPAPenetration& epa)
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

		return  true;
	}

	static bool ConstructSupportFace(Geometry* GeomA, Geometry* GeomB, const Vector3d& penetration_normal, SupportFace &Face)
	{
		SupportFace FaceA, FaceB;
		GeomA->GetSupportFace_WorldSpace(-penetration_normal, FaceA);
		GeomB->GetSupportFace_WorldSpace(penetration_normal, FaceB);

		float mSpeculativeContactDistance = 0.02f;
		bool succ = ClipPolygonAgainPolygon3D(FaceA.GetData(), FaceA.GetSize(), FaceB.GetData(), FaceB.GetSize(), penetration_normal, mSpeculativeContactDistance, Face.GetData(), Face.GetSizeData(), nullptr, nullptr);
		return succ;
	}

	void ConstructManifols(Geometry* GeomA, Geometry* GeomB, EPAPenetration& epa, std::vector<ContactManifold>* manifolds)
	{
		manifolds->push_back(ContactManifold());

		// http://allenchou.net/2013/12/game-physics-contact-generation-epa/
		Vector3d w0 = Vector3d::Zero();
		for (int i = 0; i < epa.result.dimension; ++i)
		{
			Vector3d pi = GeomA->GetSupport_WorldSpace(epa.result.v[i].dir) * epa.result.w[i];
			w0 = w0 + pi;
		}

		SupportFace ContactFace;
		if (mUseContactFace)
		{
			ConstructSupportFace(GeomA, GeomB, epa.penetration_normal, ContactFace);
		}

		const Matrix4d& invWorld = GeomA->GetInverseWorldMatrix();
		for (int i = -1; i < ContactFace.GetSize(); ++i)
		{
			Vector3d p0 = i == -1 ? w0 : ContactFace[i];
			Contact contact;
			contact.PositionLocalA = invWorld * p0;
			Vector3d p2 = p0 - epa.penetration_normal * epa.penetration_depth;
			contact.PositionLocalB = invWorld * p2;
			contact.PositionWorldA = p0;
			contact.PositionWorldB = p2;
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

			manifolds->back().AddNewContact(GeomA, GeomB, contact);
		}

		return;
	}

private:
	bool mUseContactFace;
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

	virtual void CollisionDetection(std::vector<OverlapPair>& overlaps, std::vector<ContactManifold>* manifolds) override final
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

	virtual void CollisionDetection(std::vector<OverlapPair>& overlaps, std::vector<ContactManifold>* manifolds) override final
	{
		return;
	}
};

NarrowPhase* NarrowPhase::Create_GJKEPA()
{
	return new NarrowPhase_GJKEPA();
}
