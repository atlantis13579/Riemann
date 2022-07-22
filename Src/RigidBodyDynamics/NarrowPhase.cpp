
#include "NarrowPhase.h"
#include "Contact.h"
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
		if (gjk_status != GJK_status::Inside)
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

	void ConstructManifols(Geometry* Geom1, Geometry* Geom2, EPAPenetration& epa, std::vector<ContactManifold>* manifolds)
	{
		Contact contact;

		// http://allenchou.net/2013/12/game-physics-contact-generation-epa/
		Vector3d w0 = Vector3d::Zero();
		for (int i = 0; i < epa.result.dimension; ++i)
		{
			Vector3d pi = Geom1->GetSupport_WorldSpace(epa.result.v[i].dir) * epa.result.w[i];
			w0 = w0 + pi;
		}
		const Matrix4d& invWorld = Geom1->GetInverseWorldMatrix();
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

		manifolds->push_back(ContactManifold());
		manifolds->back().AddNewContact(Geom1, Geom2, contact);
	}

	void ConstructManifold(std::vector<ContactManifold>* manifolds)
	{

	}

private:


};

NarrowPhase* NarrowPhase::Create_GJKEPA()
{
	return new NarrowPhase_GJKEPA();
}
