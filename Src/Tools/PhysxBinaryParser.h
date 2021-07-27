#pragma once

#include <vector>

enum PxType
{
	eUNDEFINED,

	eHEIGHTFIELD,
	eCONVEX_MESH,
	eTRIANGLE_MESH_BVH33,
	eTRIANGLE_MESH_BVH34,

	eRIGID_DYNAMIC,
	eRIGID_STATIC,
	eSHAPE,
	eMATERIAL,
	eCONSTRAINT,
	eAGGREGATE,
	eARTICULATION,
	eARTICULATION_REDUCED_COORDINATE,
	eARTICULATION_LINK,
	eARTICULATION_JOINT,
	eARTICULATION_JOINT_REDUCED_COORDINATE,
	ePRUNING_STRUCTURE,
	eBVH_STRUCTURE,

	ePHYSX_CORE_COUNT,
	eFIRST_PHYSX_EXTENSION = 256,
	eFIRST_VEHICLE_EXTENSION = 512,
	eFIRST_USER_EXTENSION = 1024
};

class PhysxObjectDeserializer
{
public:
	virtual void*	CreateObject(unsigned char*& address, PxType Type) = 0;
	virtual void	AddToCollection(void *pObject) = 0;
};

class PhysxBinaryParser
{
public:
	static bool ParseCollectionFromBinary(const char* Filename, PhysxObjectDeserializer *pDeserializer);
};
