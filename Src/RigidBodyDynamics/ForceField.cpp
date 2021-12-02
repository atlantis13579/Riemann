
#include "ForceField.h"
#include "RigidBody.h"

ForceField::ForceField(const Vector3d& Force) : DenseTensorField3d(Force)
{

}

ForceField::~ForceField()
{

}

bool ForceField::ApplyForce(RigidBodyDynamic* Rigid)
{
	Vector3d Force = DenseTensorField3d::GetTensorByPosition(Rigid->X);
	Rigid->ApplyForce(Force);
	return true;
}
