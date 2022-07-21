
#include "NarrowPhase.h"
#include "Contact.h"
#include "GeometryObject.h"
#include "GeometryDifference.h"
#include "EPAPenetration.h"
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

	virtual void CollisionDetection(std::vector<OverlapPair>& overlaps, std::vector<ContactManifold>* contact) override final
	{
		contact->clear();
		for (size_t i = 0; i < overlaps.size(); ++i)
		{
			ContactResult result;
			if (PenetrationTest(overlaps[i].Geom1, overlaps[i].Geom2, result))
			{
				ContactManifold manifold;
				manifold.AddNewContact(overlaps[i].Geom1, overlaps[i].Geom2, result);
				contact->push_back(manifold);
			}
		}
	}

	bool PenetrationTest(Geometry* Geom1, Geometry* Geom2, ContactResult& result)
	{
		GeometryDifference shape(Geom1, Geom2);
		GJKIntersection gjk;
		GJK_status gjk_status = gjk.Solve(&shape);
		if (gjk_status != GJK_status::Inside)
		{
			return false;
		}

		EPAPenetration epa;
		EPA_status epa_result = epa.Solve(gjk.result);
		if (epa_result == EPA_status::Failed || epa_result == EPA_status::FallBack)
		{
			return false;
		}

		// http://allenchou.net/2013/12/game-physics-contact-generation-epa/
		Vector3d w0 = Vector3d::Zero();
		for (int i = 0; i < epa.result.dimension; ++i)
		{
			Vector3d pi = shape.Support1(epa.result.v[i].dir) * epa.result.w[i];
			w0 = w0 + pi;
		}
		const Matrix4d& invWorld = Geom1->GetInverseWorldMatrix();
		result.PositionLocal1 = invWorld * w0;
		Vector3d p2 = w0 - epa.penetration_normal * epa.penetration_depth;
		result.PositionLocal2 = invWorld * p2;
		result.PositionWorld1 = w0;
		result.PositionWorld2 = p2;
		result.Normal = epa.penetration_normal;
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
