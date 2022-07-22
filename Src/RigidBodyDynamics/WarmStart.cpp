
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
            WarmStart::Contact(manifold.GeomA, manifold.GeomB, manifold.ContactPoints[i], dt);
        }
    }
}

void WarmStart::Contact(Geometry* GeomA, Geometry* GeomB, ContactResult& contact, float dt)
{
    RigidBody* rigidBodyA = (RigidBody*)GeomA->GetEntity();
    RigidBody* rigidBodyB = (RigidBody*)GeomB->GetEntity();

    //let normal point from a -> b
    Vector3d normal = -contact.Normal;
	Vector3d RelativePositionA = contact.PositionWorldA - GeomA->GetPosition();
	Vector3d RelativePositionB = contact.PositionWorldB - GeomB->GetPosition();

    // collision impulse
    float collisionImpulseVal = contact.SumImpulseNormal;
    Vector3d collisionImpulse = normal * collisionImpulseVal;

    RigidBodyDynamic* rigidBodyDyn1 = rigidBodyA->CastDynamic();
	if (rigidBodyDyn1 && !rigidBodyDyn1->Sleep)
	{
        rigidBodyDyn1->P = rigidBodyDyn1->P - collisionImpulse;
        rigidBodyDyn1->L = rigidBodyDyn1->L - collisionImpulseVal * RelativePositionA.Cross(normal);
    }

	RigidBodyDynamic* rigidBodyDyn2 = rigidBodyB->CastDynamic();
    if (rigidBodyDyn2 && !rigidBodyDyn2->Sleep)
    {
        rigidBodyDyn2->P = rigidBodyDyn2->P + collisionImpulse;
        rigidBodyDyn2->L = rigidBodyDyn2->L - collisionImpulseVal * RelativePositionB.Cross(normal);
    }

    // friction impulse
    Vector3d tangent = contact.Tangent1;
    float frictionImpulseVal = contact.SumImpulseTangent1;
    Vector3d frictionImpulse = tangent * frictionImpulseVal;

    if (rigidBodyDyn1 && !rigidBodyDyn1->Sleep)
    {
        rigidBodyDyn1->P = rigidBodyDyn1->P - frictionImpulse;
        rigidBodyDyn1->L = rigidBodyDyn1->L - frictionImpulseVal * RelativePositionA.Cross(tangent);
    }

    if (rigidBodyDyn2 && !rigidBodyDyn2->Sleep)
    {
        rigidBodyDyn2->P = rigidBodyDyn2->P + frictionImpulse;
        rigidBodyDyn2->L = rigidBodyDyn2->L - frictionImpulseVal * RelativePositionB.Cross(tangent);
    }
}

