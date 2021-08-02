
#include "NarrowPhase.h"
#include "Contact.h"
#include "GeometryObject.h"
#include "EPA.h"
#include "GJK.h"

class GeometrySum : public MinkowskiSum
{
public:
	GeometrySum() {}
	GeometrySum(Geometry* _g1, Geometry* _g2)
	{
		Geom1 = _g1;
		Geom2 = _g2;
	}

	Geometry* Geom1;
	Geometry* Geom2;

	inline Vector3d Support1(const Vector3d& Dir)
	{
		return Geom1->GetSupport_WorldSpace(Dir);
	}

	inline Vector3d Support2(const Vector3d& Dir)
	{
		return Geom2->GetSupport_WorldSpace(Dir);
	}

	virtual Vector3d Support(const Vector3d& Dir) override
	{
		Vector3d support1 = Support1(Dir);
		Vector3d support2 = Support2(-Dir);
		Vector3d diff = support1 - support2;
		return diff;
	}
};

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
		Vector3d position1 = Geom1->GetBoundingVolume_WorldSpace().GetCenter();
		Vector3d position2 = Geom2->GetBoundingVolume_WorldSpace().GetCenter();
		Vector3d guess = position1 - position2;

		// result
		result.WitnessLocal1 = result.WitnessLocal2 = result.WitnessWorld1 = result.WitnessWorld2 = Vector3d::Zero();
		result.status = ContactResult::Separated;

		GeometrySum shape(Geom1, Geom2);

		GJK gjk;
		GJK_status gjk_status = gjk.Solve(&shape, -guess);

		switch (gjk_status)
		{

		case GJK_status::Inside:
		{
			EPA epa;
			EPA_status epa_status = epa.Solve(gjk.cs, &shape, -guess);
			if (epa_status != EPA_status::Failed)
			{
				// http://allenchou.net/2013/12/game-physics-contact-generation-epa/

				Vector3d w0 = Vector3d(0, 0, 0);
				for (int i = 0; i < epa.m_result.dimension; ++i)
				{
					w0 = w0 + shape.Support1(epa.m_result.v[i].d) * epa.m_result.w[i];
				}
				Matrix4d invWorld = Geom1->GetInverseWorldMatrix();
				result.status = ContactResult::Penetrating;
				result.WitnessLocal1 = invWorld * w0;
				Vector3d secondObjectPointInFirstObject = w0 - epa.m_normal * epa.m_depth;
				result.WitnessLocal2 = invWorld * secondObjectPointInFirstObject;
				result.WitnessWorld1 = w0;
				result.WitnessWorld2 = secondObjectPointInFirstObject;
				result.Normal = epa.m_normal;
				result.PenetrationDistance = epa.m_depth;
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
				result.Tangent1;
				result.Tangent2 = CrossProduct(result.Normal, result.Tangent1);
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
