
#include "PhysxBinaryParser.h"
#include "Serialization.h"
#include "../Collision/GeometryObject.h"
#include "../CollisionPrimitive/ConvexMesh.h"
#include "../Collision/TriangleMesh.h"
#include "../Collision/RTree.h"
#include "../Maths/Box3d.h"
#include "../Maths/Transform.h"
#include "../Maths/Quaternion.h"

#include <assert.h>
#include <stdio.h>
#include <unordered_map>
#include <vector>

typedef int64_t PxI64;
typedef uint64_t PxU64;
typedef int32_t PxI32;
typedef uint32_t PxU32;
typedef int16_t PxI16;
typedef uint16_t PxU16;
typedef int8_t PxI8;
typedef uint8_t PxU8;
typedef float PxF32;
typedef double PxF64;
typedef float PxReal;
typedef Vector3d PxVec3;

typedef unsigned long long PxSerialObjectId;

struct Collections
{
	std::unordered_map<PxSerialObjectId, void*>		mIds;
	std::unordered_map<void*, PxSerialObjectId>		mObjects;
	std::unordered_map<void*, int>					mClass;
};

#include "PhysxFormat_41.hpp"
#define physx	PhysxFormat_41

class PhysxBinaryParser
{
public:
	static Geometry* CreateTriangleMesh(const physx::PxTriangleMeshGeometry* physxObj)
	{
		const physx::PxRTreeTriangleMesh* Mesh = (const physx::PxRTreeTriangleMesh*)physxObj->triangleMesh;

		Geometry *Geom = GeometryFactory::CreateTriangleMesh(Mesh->mAABB.Center);
		TriangleMesh* TriMesh = (TriangleMesh*)Geom->GetShapeGeometry();
		TriMesh->SetData(Mesh->mVertices, Mesh->mTriangles, Mesh->mNbVertices, Mesh->mNbTriangles, Mesh->Is16BitIndices());
		TriMesh->BoundingVolume = Mesh->mAABB.GetAABB();

		RTree* tree = TriMesh->CreateEmptyRTree();
		memcpy(tree, &Mesh->mRTree, sizeof(RTree));
		void* pMem = TriMesh->AllocMemory(Mesh->mRTree.mTotalPages * sizeof(RTreePage), 128);
		memcpy(pMem, Mesh->mRTree.mPages, Mesh->mRTree.mTotalPages * sizeof(RTreePage));
		tree->mPages = (RTreePage*)pMem;
		return Geom;
	}

	static Geometry* CreateHeightField(const physx::PxHeightFieldGeometry* physxObj)
	{
		const physx::HeightField* hf = physxObj->heightField;
		const physx::PxHeightFieldSample*	samples = hf->mData.samples;
		const unsigned int					nCols = hf->mData.columns;
		const unsigned int					nRows = hf->mData.rows;

		Vector3d Scale = Vector3d(physxObj->rowScale, physxObj->heightScale, physxObj->columnScale);

		std::vector<Vector3d>	Verties;
		Verties.resize(nRows * nCols);
		for (unsigned int i = 0; i < nRows; i++)
		for (unsigned int j = 0; j < nCols; j++)
		{
			Verties[i * nCols + j] = Vector3d(1.0f * i, samples[j + (i * nCols)].height, 1.0f * j) * Scale;
		}

		std::vector<unsigned short>	Indices;
		assert(Verties.size() < 65535);
		Indices.resize((nCols - 1) * (nRows - 1) * 2 * 3);
		int nFaces = 0;

		for (unsigned int i = 0; i < (nCols - 1); ++i)
		for (unsigned int j = 0; j < (nRows - 1); ++j)
		{
			PxU8 tessFlag = samples[i + j * nCols].materialIndex0 & 0x80;
			PxU32 i0 = j * nCols + i;
			PxU32 i1 = j * nCols + i + 1;
			PxU32 i2 = (j + 1) * nCols + i;
			PxU32 i3 = (j + 1) * nCols + i + 1;
			// i2---i3
			// |    |
			// |    |
			// i0---i1
			const int PxHeightFieldMaterial_eHOLE = 127;
			PxU32 mat0 = hf->getTriangleMaterialIndex((j * nCols + i) * 2);
			PxU32 mat1 = hf->getTriangleMaterialIndex((j * nCols + i) * 2 + 1);
			if (mat0 != PxHeightFieldMaterial_eHOLE)
			{
				Indices[3 * nFaces + 0] = i2;
				Indices[3 * nFaces + 1] = i0;
				Indices[3 * nFaces + 2] = tessFlag ? i3 : i1;
				nFaces++;
			}
			if (mat1 != PxHeightFieldMaterial_eHOLE)
			{
				Indices[3 * nFaces + 0] = i3;
				Indices[3 * nFaces + 1] = tessFlag ? i0 : i2;
				Indices[3 * nFaces + 2] = i1;
				nFaces++;
			}
		}

		Geometry* Geom = GeometryFactory::CreateTriangleMesh(hf->mData.mAABB.Center * Scale);
		TriangleMesh* TriMesh = (TriangleMesh*)Geom->GetShapeGeometry();
		TriMesh->SetData(&Verties[0], &Indices[0], (unsigned int)Verties.size(), nFaces, true);
		// TriMesh->AddAABB(ce.Center - ce.Extent, ce.Center + ce.Extent);

		return Geom;
	}

	static Geometry* CreateConvexMesh(const physx::PxConvexMeshGeometry* physxObj)
	{
		const physx::PxConvexMesh* Mesh = physxObj->convexMesh;

		Geometry* Geom = GeometryFactory::CreateConvexMesh(Mesh->mHullData.mAABB.Center);
		ConvexMesh* ConvMesh = (ConvexMesh*)Geom->GetShapeGeometry();

		for (int i = 0; i < Mesh->mHullData.mNbPolygons; ++i)
		{
			ConvMesh->AddFace(Mesh->mHullData.mPolygons[i].mPlane);
		}
		ConvMesh->SetVerties(Mesh->mHullData.getVerts(), Mesh->mHullData.mNbHullVertices);

		assert(Mesh->mHullData.getVerticesByEdges16());
		ConvMesh->SetEdges(Mesh->mHullData.getVerticesByEdges16(), Mesh->mHullData.mNbEdges & ~0x8000);
		assert(ConvMesh->VerifyIndices());

		ConvMesh->Inertia = Mesh->mInertia;
		ConvMesh->CenterOfMass = Mesh->mHullData.mCenterOfMass;
		ConvMesh->BoundingVolume = Mesh->mHullData.mAABB.GetAABB();

		assert(ConvMesh->EulerNumber() == 2);
		assert(ConvMesh->NumVertices == ConvMesh->Verties.size());
		assert(ConvMesh->NumEdges * 2 == ConvMesh->Edges.size());
		assert(ConvMesh->NumFaces == ConvMesh->Faces.size());

		return Geom;
	}

	static Geometry* CreateShape(const physx::NpShape *shape)
	{

		Geometry* Geom = nullptr;
		int Type = shape->getGeomType();
		if (Type == physx::eSPHERE)
		{
			physx::PxSphereGeometry* sphere = (physx::PxSphereGeometry*)shape->mShape.mShape.mCore.geometry.mGeometry.sphere;

			Geom = nullptr;
		}
		else if (Type == physx::ePLANE)
		{
			physx::PxPlaneGeometry* plane = (physx::PxPlaneGeometry*)shape->mShape.mShape.mCore.geometry.mGeometry.plane;

			Geom = nullptr;
		}
		else if (Type == physx::eCAPSULE)
		{
			physx::PxCapsuleGeometry* capsule = (physx::PxCapsuleGeometry*)shape->mShape.mShape.mCore.geometry.mGeometry.capsule;

			Geom = nullptr;
		}
		else if (Type == physx::eBOX)
		{
			physx::PxBoxGeometry* box = (physx::PxBoxGeometry*)shape->mShape.mShape.mCore.geometry.mGeometry.box;

			Geom = nullptr;
		}
		else if (Type == physx::eCONVEXMESH)
		{
			physx::PxConvexMeshGeometry* convex = (physx::PxConvexMeshGeometry*)shape->mShape.mShape.mCore.geometry.mGeometry.convex;
			return CreateConvexMesh(convex);
		}
		else if (Type == physx::eTRIANGLEMESH)
		{
			physx::PxTriangleMeshGeometry* pxMesh = (physx::PxTriangleMeshGeometry*)shape->mShape.mShape.mCore.geometry.mGeometry.mesh;
			return CreateTriangleMesh(pxMesh);
		}
		else if (Type == physx::eHEIGHTFIELD)
		{
			physx::PxHeightFieldGeometry* hightfield = (physx::PxHeightFieldGeometry*)shape->mShape.mShape.mCore.geometry.mGeometry.heightfield;
			return CreateHeightField(hightfield);
		}
		else
		{
			assert(false);
		}

		return Geom;
	}

	static void CreateGeometryObjects(void *px, int classType, std::vector<Geometry*> *objs)
	{
		if (classType == physx::eMATERIAL)
		{
			physx::NpMaterial *material = (physx::NpMaterial*)px;

			return;
		}
		else if (classType == physx::eRIGID_STATIC)
		{
			physx::NpRigidStatic* rigid = (physx::NpRigidStatic*)px;
			int nShapes = rigid->GetNumShapes();
			physx::NpShape* const* pShades = rigid->GetShapes();
			for (int i = 0; i < nShapes; ++i)
			{
				Geometry* p = CreateShape(pShades[i]);
				if (p)
				{
					p->SetPosition(rigid->mRigidStatic.mStatic.mCore.body2World.p);
					p->SetRotationQuat(rigid->mRigidStatic.mStatic.mCore.body2World.q);
					objs->push_back(p);
				}
			}
			
			return;
		}
		else if (classType == physx::eRIGID_DYNAMIC)
		{
			physx::NpRigidDynamic* rigid = (physx::NpRigidDynamic*)px;

			return;
		}
		return;
	}
};

bool LoadPhysxBinary(const char* Filename, std::vector<Geometry*>* GeometryList)
{
	FILE* fp = fopen(Filename, "rb");
	if (fp == nullptr)
	{
		return false;
	}

	std::vector<char> buffer;
	fseek(fp, 0, SEEK_END);
	unsigned long long filesize = (unsigned long long)ftell(fp);
	fseek(fp, 0, 0);
	buffer.resize(filesize + 127);
	void* p = AlignMemory(&buffer[0], 128);
	fread(p, 1, filesize, fp);
	fclose(fp);

	Collections collection;
	physx parser;
	if (!parser.DeserializeFromBuffer(p, collection))
	{
		return false;
	}

	size_t Count = collection.mClass.size();
	for (auto it : collection.mClass)
	{
		PhysxBinaryParser::CreateGeometryObjects(it.first, it.second, GeometryList);
	}

	return true;
}