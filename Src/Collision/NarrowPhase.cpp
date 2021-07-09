
#include "NarrowPhase.h"
#include "GeometryObject.h"
#include "EPA.h"
#include "GJK.h"

struct PenetrationResults
{
	enum eStatus
	{
		Separated,   // Shapes doesnt penetrate	
		Penetrating, // Shapes are penetrating	
		GJK_Failed,  // GJK phase fail, no big issue, shapes are probably just touching
		EPA_Failed   // EPA phase fail, bigger problem, need to save parameters, and debug
	} status;

	Vector3d witnessInGlobal[2];
	Vector3d witnessesInFirstLocal[2];
	Vector3d normal;
	float distance;
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

	virtual void CollisionDetection(std::vector<OverlapPair>& overlaps, std::vector<ContactPair>* collides) override
	{
		collides->clear();
		for (size_t i = 0; i < overlaps.size(); ++i)
		{
			PenetrationResults result;
			if (Penetration(overlaps[i].Geom1, overlaps[i].Geom2, result))
			{

			}
		}
	}

	bool Penetration(Geometry* Geom1, Geometry* Geom2, PenetrationResults& result)
	{
		Vector3d position1 = Geom1->GetPositionWorld();
		Vector3d position2 = Geom2->GetPositionWorld();
		Vector3d guess = position1 - position2;

		// result
		result.witnessesInFirstLocal[0] = result.witnessesInFirstLocal[1] = result.witnessInGlobal[0] = result.witnessInGlobal[1] = Vector3d::Zero();
		result.status = PenetrationResults::Separated;

		Minkowski shape(Geom1, Geom2);

		GJK gjk;
		GJK_status gjk_status = gjk.Evaluate(&shape, guess * -1);

		switch (gjk_status)
		{

		case GJK_status::Inside:
		{
			EPA epa;
			EPA_status epa_status = epa.Evaluate(gjk, guess * -1);
			if (epa_status != EPA_status::Failed)
			{
				Vector3d w0 = Vector3d(0, 0, 0);
				for (int i = 0; i < epa.m_result.dimension; ++i)
				{
					w0 = w0 + shape.Support1(epa.m_result.c[i]->d) * epa.m_result.p[i];
				}
				Matrix4d wtrs1 = Geom1->GetInverseWorldMatrix();
				result.status = PenetrationResults::Penetrating;
				result.witnessesInFirstLocal[0] = Transform::TransformPosition(wtrs1, w0);
				Vector3d secondObjectPointInFirstObject = w0 - epa.m_normal * epa.m_depth;
				result.witnessesInFirstLocal[1] = Transform::TransformPosition(wtrs1, secondObjectPointInFirstObject);
				result.witnessInGlobal[0] = w0;
				result.witnessInGlobal[1] = secondObjectPointInFirstObject;
				result.normal = epa.m_normal;
				result.distance = epa.m_depth;
				return true;
			}
			else
			{
				result.status = PenetrationResults::EPA_Failed;
			}

		}
		case GJK_status::Failed:
		{
			result.status = PenetrationResults::GJK_Failed;
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