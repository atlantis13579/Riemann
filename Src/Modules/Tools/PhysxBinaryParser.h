#pragma once

#include <vector>
#include "../../Maths/Vector3.h"

namespace Riemann
{
class RigidBody;
class Geometry;
class IBinaryData;

bool	LoadPhysxBinary(const char* Filename, std::vector<RigidBody*> *bodies, std::vector<Geometry*>* geoms);
IBinaryData*	LoadPhysxBinaryMmap(const char* Filename, std::vector<RigidBody*>* bodies, std::vector<Geometry*>* geoms);
bool	LoadPhysxBinaryTriangles(const char* Filename, std::vector<Vector3>& vertices, std::vector<int>& indices);

}
