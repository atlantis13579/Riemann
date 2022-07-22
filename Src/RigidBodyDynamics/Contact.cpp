
#include <assert.h>
#include "Contact.h"
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

	uint8_t mask = 0;

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
