
#include "CollidingContact.h"
#include "WarmStart.h"
#include "RigidBody.h"
#include "../Collision/GeometryObject.h"

// http://allenchou.net/2014/01/game-physics-stability-warm-starting/

namespace Riemann
{
    void WarmStart::ApplyVelocityConstraint(const std::vector<Geometry*>& geoms, std::vector<ContactManifold*>& manifolds, float dt)
    {
        for (size_t i = 0; i < manifolds.size(); ++i)
        {
            ContactManifold* manifold = manifolds[i];
            for (int i = 0; i < manifold->NumContactPointCount; i++)
            {
                RigidBody* BodyA = geoms[manifold->indexA]->GetParent<RigidBody>();
                RigidBody* BodyB = geoms[manifold->indexB]->GetParent<RigidBody>();
                WarmStart::Apply(BodyA, BodyB, manifold->ContactPoints[i], dt);
            }
        }
    }

    void WarmStart::Apply(RigidBody* BodyA, RigidBody* BodyB, Contact& contact, float dt)
    {
        // TODO, not implemented
        return;

        /*
        //let normal point from a -> b
        Vector3 normal = -contact.Normal;

        // collision impulse
        float collisionImpulseVal = contact.totalImpulseNormal;
        Vector3 collisionImpulse = normal * collisionImpulseVal;

        RigidBodyDynamic* rigidBodyDyn1 = BodyA->CastDynamic();
        if (rigidBodyDyn1 && !rigidBodyDyn1->Sleeping)
        {
            rigidBodyDyn1->P = rigidBodyDyn1->P - collisionImpulse;
            rigidBodyDyn1->L = rigidBodyDyn1->L - collisionImpulseVal * contact.PositionLocalA.Cross(normal);
        }

        RigidBodyDynamic* rigidBodyDyn2 = BodyB->CastDynamic();
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
        */
    }

}