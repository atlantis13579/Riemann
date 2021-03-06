
#include "CollidingContact.h"
#include "WarmStart.h"
#include "RigidBody.h"
#include "../Collision/GeometryObject.h"

// http://allenchou.net/2014/01/game-physics-stability-warm-starting/

void WarmStart::ApplyVelocityConstraint(std::vector<ContactManifold*>& manifolds, float dt)
{
    for (size_t i = 0; i < manifolds.size(); ++i)
    {
        ContactManifold* manifold = manifolds[i];
        for (int i = 0; i < manifold->NumContactPointCount; i++)
        {
            WarmStart::Apply(manifold->GeomA, manifold->GeomB, manifold->ContactPoints[i], dt);
        }
    }
}

void WarmStart::Apply(Geometry* GeomA, Geometry* GeomB, Contact& contact, float dt)
{
    RigidBody* rigidBodyA = GeomA->GetParent<RigidBody>();
    RigidBody* rigidBodyB = GeomB->GetParent<RigidBody>();

    //let normal point from a -> b
    Vector3 normal = -contact.Normal;

    // collision impulse
    float collisionImpulseVal = contact.totalImpulseNormal;
    Vector3 collisionImpulse = normal * collisionImpulseVal;

    RigidBodyDynamic* rigidBodyDyn1 = rigidBodyA->CastDynamic();
	if (rigidBodyDyn1 && !rigidBodyDyn1->Sleeping)
	{
        rigidBodyDyn1->P = rigidBodyDyn1->P - collisionImpulse;
        rigidBodyDyn1->L = rigidBodyDyn1->L - collisionImpulseVal * contact.PositionLocalA.Cross(normal);
    }

	RigidBodyDynamic* rigidBodyDyn2 = rigidBodyB->CastDynamic();
    if (rigidBodyDyn2 && !rigidBodyDyn2->Sleeping)
    {
        rigidBodyDyn2->P = rigidBodyDyn2->P + collisionImpulse;
        rigidBodyDyn2->L = rigidBodyDyn2->L - collisionImpulseVal * contact.PositionLocalB.Cross(normal);
    }

    // friction impulse
    Vector3 tangent = contact.Tangent;
    float frictionImpulseVal = contact.totalImpulseTangent;
    Vector3 frictionImpulse = tangent * frictionImpulseVal;

    if (rigidBodyDyn1 && !rigidBodyDyn1->Sleeping)
    {
        rigidBodyDyn1->P = rigidBodyDyn1->P - frictionImpulse;
        rigidBodyDyn1->L = rigidBodyDyn1->L - frictionImpulseVal * contact.PositionLocalA.Cross(tangent);
    }

    if (rigidBodyDyn2 && !rigidBodyDyn2->Sleeping)
    {
        rigidBodyDyn2->P = rigidBodyDyn2->P + frictionImpulse;
        rigidBodyDyn2->L = rigidBodyDyn2->L - frictionImpulseVal * contact.PositionLocalB.Cross(tangent);
    }
}

