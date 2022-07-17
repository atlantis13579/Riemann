
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
		
		// result
		result.WitnessLocal1 = result.WitnessLocal2 = result.WitnessWorld1 = result.WitnessWorld2 = Vector3d::Zero();
		result.status = ContactResult::Separated;
		
		switch (gjk_status)
		{

		case GJK_status::Inside:
		{
			EPA epa;
			EPA_status epa_status = epa.Solve(gjk.simplex, &shape, -guess);
			if (epa_status != EPA_status::Failed)
			{
				// http://allenchou.net/2013/12/game-physics-contact-generation-epa/

				Vector3d w0 = Vector3d::Zero();
				for (int i = 0; i < epa.simplex.dimension; ++i)
				{
					w0 = w0 + shape.Support1(epa.simplex.v[i].d) * epa.simplex.w[i];
				}
				Matrix4d invWorld = Geom1->GetInverseWorldMatrix();
				result.status = ContactResult::Penetrating;
				result.WitnessLocal1 = invWorld * w0;
				Vector3d secondObjectPointInFirstObject = w0 - epa.normal * epa.penetration_depth;
				result.WitnessLocal2 = invWorld * secondObjectPointInFirstObject;
				result.WitnessWorld1 = w0;
				result.WitnessWorld2 = secondObjectPointInFirstObject;
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
				// result.Tangent1;
				result.Tangent2 = CrossProduct(result.Normal, result.Tangent1);
				result.R1 = result.WitnessWorld1 - Geom1->GetPosition();
				result.R2 = result.WitnessWorld2 - Geom2->GetPosition();
				return true;
			}
			else
			{
				result.status = ContactResult::EPA_Failed;
			}

		}
		case GJK_status::Failed:
		{
			result.status = ContactResult::GJK_Failed;
		}
		default:
		{
			break;
		}

		}
		return false;
	}

private:


};

NarrowPhase* NarrowPhase::Create_GJKEPA()
{
	return new NarrowPhase_GJKEPA();
}
