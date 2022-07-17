
#include "WarmStart.h"
#include "RigidBody.h"
#include "../Collision/GeometryObject.h"

// http://allenchou.net/2014/01/game-physics-stability-warm-starting/

void WarmStart::Manifolds(std::vector<ContactManifold>& manifolds, float dt)
{
    for (size_t i = 0; i < manifolds.size(); ++i)
    {
        ContactManifold& manifold = manifolds[i];
        for (int i = 0; i < manifold.NumContactPointCount; i++)
        {
            WarmStart::Contact(manifold.Geom1, manifold.Geom2, manifold.ContactPoints[i], dt);
        }
    }
}

void WarmStart::Contact(Geometry* Geom1, Geometry* Geom2, ContactResult& contact, float dt)
{
    RigidBody* rigidBody1 = (RigidBody*)Geom1->GetEntity();
    RigidBody* rigidBody2 = (RigidBody*)Geom2->GetEntity();

    //let normal point from a -> b
    Vector3d normal = -contact.Normal;

    Vector3d relativePos1 = contact.WitnessWorld1 - rigidBody1->X;
    Vector3d relativePos2 = contact.WitnessWorld2 - rigidBody2->X;

    // collision impulse
    float collisionImpulseVal = contact.ImpulseNormal;
    Vector3d collisionImpulse = normal * collisionImpulseVal;

    if (!rigidBody1->Static)
    {
		RigidBodyDynamic* rigidBodyDyn1 = (RigidBodyDynamic*)rigidBody1;
		if (!rigidBodyDyn1->Sleep)
		{
            rigidBodyDyn1->P = rigidBodyDyn1->P - collisionImpulse;
            rigidBodyDyn1->L = rigidBodyDyn1->L - collisionImpulseVal * relativePos1.Cross(normal);
		}
    }

	if (!rigidBody2->Static)
	{
		RigidBodyDynamic* rigidBodyDyn2 = (RigidBodyDynamic*)rigidBody2;
        if (!rigidBodyDyn2->Sleep)
        {
            rigidBodyDyn2->P = rigidBodyDyn2->P + collisionImpulse;
            rigidBodyDyn2->L = rigidBodyDyn2->L - collisionImpulseVal * relativePos2.Cross(normal);
        }
    }

    // friction impulse
    Vector3d tangent = contact.Tangent1;
    float frictionImpulseVal = contact.ImpulseTangent1;
    Vector3d frictionImpulse = tangent * frictionImpulseVal;

	if (!rigidBody1->Static)
	{
		RigidBodyDynamic* rigidBodyDyn1 = (RigidBodyDynamic*)rigidBody1;
        if (!rigidBodyDyn1->Sleep)
        {
            rigidBodyDyn1->P = rigidBodyDyn1->P - frictionImpulse;
            rigidBodyDyn1->L = rigidBodyDyn1->L - frictionImpulseVal * relativePos1.Cross(tangent);
        }
    }

	if (!rigidBody2->Static)
	{
		RigidBodyDynamic* rigidBodyDyn2 = (RigidBodyDynamic*)rigidBody2;
        if (!rigidBodyDyn2->Sleep)
        {
            rigidBodyDyn2->P = rigidBodyDyn2->P + frictionImpulse;
            rigidBodyDyn2->L = rigidBodyDyn2->L - frictionImpulseVal * relativePos2.Cross(tangent);
        }
    }
}

