
#include "NarrowPhase.h"
#include "Contact.h"
#include "GeometryObject.h"
#include "GeometryDifference.h"
#include "EPA.h"
#include "GJK.h"

class NarrowPhase_GJKEPA : public NarrowPhase
{
public:
	NarrowPhase_GJKEPA()
	{

	}
	virtual ~NarrowPhase_GJKEPA()
	{

	}

	virtual void CollisionDetection(std::vector<OverlapPair>& overlaps, std::vector<ContactManifold>* contact) override
	{
		contact->clear();
		for (size_t i = 0; i < overlaps.size(); ++i)
		{
			ContactResult result;
			if (Penetration(overlaps[i].Geom1, overlaps[i].Geom2, result))
			{
				ContactManifold manifold;
				manifold.AddNewContact(overlaps[i].Geom1, overlaps[i].Geom2, result);
				contact->push_back(manifold);
			}
		}
	}

	bool Penetration(Geometry* Geom1, Geometry* Geom2, ContactResult& result)
	{
		GeometryDifference shape(Geom1, Geom2);
		Vector3d guess = shape.GetCenter();

		GJK gjk;
		GJK_status gjk_status = gjk.Solve(&shape, -guess);
		if (gjk_status != GJK_status::Inside)
		{
			result.status = ContactResult::GJK_Failed;
			return false;
		}

		EPA epa;
		EPA_status epa_status = epa.Solve(gjk.simplex, &shape, -guess);
		if (epa_status == EPA_status::Failed)
		{
			result.status = ContactResult::EPA_Failed;
			return false;
		}

		// http://allenchou.net/2013/12/game-physics-contact-generation-epa/
		Vector3d w0 = Vector3d::Zero();
		for (int i = 0; i < epa.simplex.dimension; ++i)
		{
			w0 = w0 + shape.Support1(epa.simplex.v[i].d) * epa.simplex.w[i];
		}
		const Matrix4d& invWorld = Geom1->GetInverseWorldMatrix();
		result.status = ContactResult::Penetrating;
		result.PositionLocal1 = invWorld * w0;
		Vector3d p2 = w0 - epa.normal * epa.penetration_depth;
		result.PositionLocal2 = invWorld * p2;
		result.PositionWorld1 = w0;
		result.PositionWorld2 = p2;
		result.Normal = epa.normal;
		result.PenetrationDepth = epa.penetration_depth;
		if (result.Normal.x >= 0.57735f)
		{
			result.Tangent1.x = result.Normal.y;
			result.Tangent1.y = -result.Normal.x;
			result.Tangent1.z = 0;
		}
		else
		{
			result.Tangent1.x = 0;
			result.Tangent1.y = result.Normal.z;
			result.Tangent1.z = -result.Normal.y;
		}
		result.Tangent2 = CrossProduct(result.Normal, result.Tangent1);
		result.RelativePosition1 = result.PositionWorld1 - Geom1->GetPosition();
		result.RelativePosition2 = result.PositionWorld2 - Geom2->GetPosition();
		return true;
	}

private:


};

NarrowPhase* NarrowPhase::Create_GJKEPA()
{
	return new NarrowPhase_GJKEPA();
}
