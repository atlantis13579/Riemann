
#include "Contact.h"

void ContactManifold::AddNewContact(Geometry* _Geom1, Geometry* _Geom2, const ContactResult& result)
{
	Geom1 = _Geom1;
	Geom2 = _Geom2;
	ContactPoints[NumContactPointCount] = result;
	NumContactPointCount++;
}
