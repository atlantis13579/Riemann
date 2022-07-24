
#include <assert.h>
#include <algorithm>
#include "Contact.h"
#include "../Core/StaticArray.h"
#include "../CollisionPrimitive/Segment3d.h"
#include "../CollisionPrimitive/Triangle3d.h"

void ContactManifold::AddNewContact(Geometry* _Geom1, Geometry* _Geom2, const Contact& result)
{
	GeomA = _Geom1;
	GeomB = _Geom2;
	ContactPoints[NumContactPointCount] = result;
	NumContactPointCount++;

	if (NumContactPointCount > MAX_CONTACT_POINTS)
	{
		MergeManifold();
	}
}

bool ContactManifold::MergeManifold()
{
	assert(NumContactPointCount == MAX_CONTACT_POINTS + 1);

	const float cMinDistanceSq = 1e-6f;
	StaticArray<Vector3d, MAX_CONTACT_POINTS + 1> projected;
	StaticArray<float, MAX_CONTACT_POINTS + 1> penetration_depth_sq;
	for (int i = 0; i < NumContactPointCount; ++i)
	{
		Contact* contact = &ContactPoints[i];

		const Vector3d& local1 = contact->PositionLocalA;
		projected.Push(local1 - local1.Dot(contact->Normal) * contact->Normal);

		const Vector3d& w1 = contact->PositionWorldA;
		const Vector3d& w2 = contact->PositionWorldB;
		penetration_depth_sq.Push(std::max(cMinDistanceSq, (w1 - w2).SquareLength()));
	}

	int point0 = -1, point1 = -1, point2 = -1, point3 = -1;

	float max_val = -FLT_MAX;
	for (int i = 0; i < NumContactPointCount; ++i)
	{
		Contact* contact = &ContactPoints[i];
		const float val = std::max(cMinDistanceSq, projected[i].SquareLength()) * penetration_depth_sq[i];
		if (max_val < val)
		{
			max_val = val;
			point0 = i;
		}
	}

	// find the contact farthest from c0
	max_val = -FLT_MAX;
	for (int i = 0; i < NumContactPointCount; ++i)
	{
		Contact* contact = &ContactPoints[i];
		if (point0 == i)
			continue;
		const float val = (contact->PositionWorldA - projected[point0]).SquareLength() * penetration_depth_sq[i];
		if (val > max_val)
		{
			max_val = val;
			point1 = i;
		}
	}

	float min_val = 0.0f;
	max_val = 0.0f;

	Vector3d perp = (projected[point0] - projected[point1]).Cross(ContactPoints[0].Normal);
	for (int i = 0; i < projected.GetSize(); ++i)
	{
		if (point0 == i || point1 == i)
			continue;

		float v = perp.Dot(projected[i] - projected[point0]);
		if (v < min_val)
		{
			min_val = v;
			point2 = i;
		}
		else if (v > max_val)
		{
			max_val = v;
			point3 = i;
		}
	}


	assert(point0 != point1 && point0 != point2 && point0 != point3);
	assert(point1 != point2 && point1 != point3);
	assert(point2 != point3);

	for (int i = 0; i < NumContactPointCount; ++i)
	{
		if (i == point0 || i == point1 || i == point2 || i == point3)
			continue;

		NumContactPointCount--;
		if (i == MAX_CONTACT_POINTS)
		{
			break;
		}

		memcpy(&ContactPoints[i], &ContactPoints[MAX_CONTACT_POINTS], sizeof(Contact));
		break;
	}

	return true;
}

ContactManifoldManager::ContactManifoldManager()
{

}

ContactManifoldManager::~ContactManifoldManager()
{

}
