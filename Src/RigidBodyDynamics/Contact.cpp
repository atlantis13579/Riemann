
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
		MergeManifold2();
	}
}

bool ContactManifold::MergeManifold()
{
	assert(NumContactPointCount == MAX_CONTACT_POINTS + 1);

	const float kMinSqrDist = 1e-6f;
	StaticArray<Vector3, MAX_CONTACT_POINTS + 1> projected;
	StaticArray<float, MAX_CONTACT_POINTS + 1> penetration_depth_sq;
	for (int i = 0; i < NumContactPointCount; ++i)
	{
		Contact* contact = &ContactPoints[i];

		const Vector3& local1 = contact->PositionLocalA;
		projected.Push(local1 - local1.Dot(contact->Normal) * contact->Normal);

		const Vector3& w1 = contact->PositionWorldA;
		const Vector3& w2 = contact->PositionWorldB;
		penetration_depth_sq.Push(std::max(kMinSqrDist, (w1 - w2).SquareLength()));
	}

	int point0 = -1, point1 = -1, point2 = -1, point3 = -1;

	float max_val = -FLT_MAX;
	for (int i = 0; i < NumContactPointCount; ++i)
	{
		Contact* contact = &ContactPoints[i];
		const float val = std::max(kMinSqrDist, projected[i].SquareLength()) * penetration_depth_sq[i];
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
		if (max_val < val)
		{
			max_val = val;
			point1 = i;
		}
	}

	float min_val = 0.0f;
	max_val = 0.0f;

	Vector3 perp = (projected[point1] - projected[point0]).Cross(ContactPoints[0].Normal);
	for (int i = 0; i < projected.GetSize(); ++i)
	{
		if (point0 == i || point1 == i)
			continue;

		const float v = perp.Dot(projected[i] - projected[point0]);
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

bool ContactManifold::MergeManifold2()
{
	assert(NumContactPointCount == MAX_CONTACT_POINTS + 1);

	// find the contact with deepest penetration depth
	Contact* c0 = nullptr;
	float max_penetration = -FLT_MAX;
	for (int i = 0; i < NumContactPointCount; ++i)
	{
		Contact* contact = &ContactPoints[i];
		if (contact->PenetrationDepth > max_penetration)
		{
			max_penetration = contact->PenetrationDepth;
			c0 = contact;
		}
	}

	// find the contact farthest from c0
	Contact* c1 = nullptr;
	float max_sqr_dist = -FLT_MAX;
	for (int i = 0; i < NumContactPointCount; ++i)
	{
		Contact* contact = &ContactPoints[i];
		if (contact == c0)
			continue;
		const float sqr_dist = (contact->PositionWorldA - c0->PositionWorldA).SquareLength();
		if (sqr_dist > max_sqr_dist)
		{
			max_sqr_dist = sqr_dist;
			c1 = contact;
		}
	}

	// find the contact farthest form segment c0 - c1
	Contact* c2 = nullptr;
	float max_sqr_dist_seq = -FLT_MAX;
	for (int i = 0; i < NumContactPointCount; ++i)
	{
		Contact* contact = &ContactPoints[i];
		if (contact == c0 || contact == c1)
			continue;
		const float dist = Segment3d::SqrDistancePointToSegment(contact->PositionWorldA, c0->PositionWorldA, c1->PositionWorldA);
		if (dist > max_sqr_dist_seq)
		{
			max_sqr_dist_seq = dist;
			c2 = contact;
		}
	}

	// find the contact farthest form triangle c0 c1 c2
	Contact* c3 = nullptr;
	float max_sqr_dist_tri = -FLT_MAX;
	for (int i = 0; i < NumContactPointCount; ++i)
	{
		Contact* contact = &ContactPoints[i];
		if (contact == c0 || contact == c1 || contact == c2)
			continue;
		const float dist = Triangle3d::SqrDistancePointToTriangle(contact->PositionWorldA, c0->PositionWorldA, c1->PositionWorldA, c2->PositionWorldA);
		if (dist > max_sqr_dist_tri)
		{
			max_sqr_dist_tri = dist;
			c3 = contact;
		}
	}

	assert(c0 != c1 && c0 != c2 && c0 != c3);
	assert(c1 != c2 && c1 != c3);
	assert(c2 != c3);

	for (int i = 0; i < NumContactPointCount; ++i)
	{
		Contact* contact = &ContactPoints[i];
		if (contact == c0 || contact == c1 || contact == c2 || contact == c3)
			continue;

		NumContactPointCount--;
		if (i == MAX_CONTACT_POINTS)
		{
			break;
		}

		memcpy(contact, &ContactPoints[MAX_CONTACT_POINTS], sizeof(Contact));
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
