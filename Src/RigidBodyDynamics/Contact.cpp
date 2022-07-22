
#include "Contact.h"

void ContactManifold::AddNewContact(Geometry* _Geom1, Geometry* _Geom2, const Contact& result)
{
	GeomA = _Geom1;
	GeomB = _Geom2;
	ContactPoints[NumContactPointCount] = result;
	NumContactPointCount++;
}

ContactManifoldManager::ContactManifoldManager()
{

}

ContactManifoldManager::~ContactManifoldManager()
{

}
