
#include "Contact.h"

void ContactManifold::AddNewContact(Geometry* _Geom1, Geometry* _Geom2, const ContactResult& result)
{
	GeomA = _Geom1;
	GeomB = _Geom2;
	ContactPoints[NumContactPointCount] = result;
	NumContactPointCount++;
}
