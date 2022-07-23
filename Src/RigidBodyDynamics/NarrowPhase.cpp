
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

	void ConstructManifols(Geometry* GeomA, Geometry* GeomB, EPAPenetration& epa, std::vector<ContactManifold>* manifolds)
	{
		manifolds->push_back(ContactManifold());

		SupportFace FaceA, FaceB;
		GeomA->GetSupportFace_WorldSpace(-epa.penetration_normal, FaceA);
		GeomB->GetSupportFace_WorldSpace(epa.penetration_normal, FaceB);

		StaticArray<Vector3d, MAX_FACE_POINTS> Contact1, Contact2;
		int c1 = 0, c2 = 0;
		float mSpeculativeContactDistance = 0.02f;
		bool succ = ClipPolygon3D(FaceA.GetData(), FaceA.GetSize(), FaceB.GetData(), FaceB.GetSize(), -epa.penetration_normal, mSpeculativeContactDistance, Contact1.GetData(), c1, Contact2.GetData(), c2);
		Contact1.SetSize(c1);
		Contact2.SetSize(c2);

		if (Contact1.GetSize() > 0 || Contact2.GetSize() > 0)
		{

			int x = 0;
		}

		// http://allenchou.net/2013/12/game-physics-contact-generation-epa/
		Vector3d w0 = Vector3d::Zero();
		for (int i = 0; i < epa.result.dimension; ++i)
		{
			Vector3d pi = GeomA->GetSupport_WorldSpace(epa.result.v[i].dir) * epa.result.w[i];
			w0 = w0 + pi;
		}

		const Matrix4d& invWorld = GeomA->GetInverseWorldMatrix();

		Contact contact;
		contact.PositionLocalA = invWorld * w0;
		Vector3d p2 = w0 - epa.penetration_normal * epa.penetration_depth;
		contact.PositionLocalB = invWorld * p2;
		contact.PositionWorldA = w0;
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

private:


};

NarrowPhase* NarrowPhase::Create_GJKEPA()
{
	return new NarrowPhase_GJKEPA();
}
