#include <assert.h>
#include <stdio.h>
#include <cstddef>
#include <unordered_map>
#include <vector>

#include "PhysxBinaryParser.h"
#include "Serialization.h"
#include "../Collision/GeometryObject.h"
#include "../CollisionPrimitive/ConvexMesh.h"
#include "../CollisionPrimitive/HeightField3d.h"
#include "../CollisionPrimitive/TriangleMesh.h"
#include "../CollisionPrimitive/MeshBVH4.h"
#include "../RigidBodyDynamics/RigidBody.h"
#include "../Maths/Base.h"
#include "../Maths/Box3d.h"
#include "../Maths/Transform.h"
#include "../Maths/Quaternion.h"

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

typedef uint64_t PxSerialObjectId;

struct PhysxCollections
{
	std::unordered_map<PxSerialObjectId, void*>		mIds;
	std::unordered_map<void*, PxSerialObjectId>		mObjects;
	std::unordered_map<void*, int>					mClass;
};

#include "PhysxFormat_34.hpp"
#define physx	PhysxFormat_34

class PhysxBinaryParser
{
public:
	static Geometry* CreateSphere(physx::PxSphereGeometry* physxObj)
	{
		return GeometryFactory::CreateSphere(Vector3d::Zero(), physxObj->radius);
	}

	static Geometry* CreatePlane(physx::PxPlaneGeometry* physxObj)
	{
		return GeometryFactory::CreatePlane(Vector3d::Zero(), Vector3d::UnitY());
	}

	static Geometry* CreateCapsule(physx::PxCapsuleGeometry* physxObj)
	{
		return GeometryFactory::CreateCapsule(Vector3d::UnitY() * -physxObj->halfHeight, Vector3d::UnitY() * physxObj->halfHeight, physxObj->radius);
	}

	static Geometry* CreateBox(physx::PxBoxGeometry* physxObj)
	{
		return GeometryFactory::CreateOBB(Vector3d::Zero(), physxObj->halfExtents);
	}

	static Geometry* CreateTriangleMesh(const physx::PxTriangleMeshGeometry* physxObj)
	{
		const physx::PxRTreeTriangleMesh* Mesh = (const physx::PxRTreeTriangleMesh*)physxObj->triangleMesh;

		Geometry *Geom = GeometryFactory::CreateTriangleMesh();
		TriangleMesh* TriMesh = Geom->GetShapeObj<TriangleMesh>();
		TriMesh->SetData(Mesh->mVertices, Mesh->mTriangles, Mesh->mNbVertices, Mesh->mNbTriangles, Mesh->Is16BitIndices());
		TriMesh->BoundingVolume = Mesh->mAABB.GetAABB();

		MeshBVH4* tree = TriMesh->CreateEmptyBVH();
		static_assert(sizeof(MeshBVH4) == sizeof(physx::RTree), "MeshBVH and RTree should have same size");
		static_assert(sizeof(BVHNodeBatch) == sizeof(physx::RTreePage), "BVHNodeBatch and RTreePage should have same size");
		memcpy(tree, &Mesh->mRTree, sizeof(MeshBVH4));
		void* pMem = TriMesh->AllocMemory(Mesh->mRTree.mTotalPages * sizeof(physx::RTreePage), 128);
		memcpy(pMem, Mesh->mRTree.mPages, Mesh->mRTree.mTotalPages * sizeof(physx::RTreePage));
		tree->BatchPtr = (BVHNodeBatch*)pMem;
		return Geom;
	}

	static Geometry* CreateHeightField(const physx::PxHeightFieldGeometry* physxObj)
	{
		const physx::HeightField* hf = physxObj->heightField;
		const physx::PxHeightFieldSample* samples = hf->mData.samples;
		const uint32_t					nCols = hf->mData.columns;
		const uint32_t					nRows = hf->mData.rows;

		TCE3<float>	ce = hf->mData.mAABB;
		Vector3d Scale = Vector3d(physxObj->rowScale, physxObj->heightScale, physxObj->columnScale);
		ce.Center *= Scale;
		ce.Extent *= Scale;
		Geometry* Geom = GeometryFactory::CreateHeightField(ce.GetAABB(), nRows, nCols);
		HeightField3d* Field = Geom->GetShapeObj<HeightField3d>();

		for (uint32_t i = 0; i < nRows; i++)
		for (uint32_t j = 0; j < nCols; j++)
		{
			Field->Heights[i * nCols + j] = samples[j + (i * nCols)].height * physxObj->heightScale;
			Field->Cells[i * nCols + j].Tessellation0 = samples[i + j * nCols].materialIndex0;
			Field->Cells[i * nCols + j].Tessellation1 = samples[i + j * nCols].materialIndex1;
		}

		return Geom;
	}

	static Geometry* CreateConvexMesh(const physx::PxConvexMeshGeometry* physxObj)
	{
		const physx::ConvexMesh* Mesh = physxObj->convexMesh;

		Geometry* Geom = GeometryFactory::CreateConvexMesh();
		ConvexMesh* ConvMesh = Geom->GetShapeObj<ConvexMesh>();

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
		assert(ConvMesh->NumVertices == ConvMesh->Vertices.size());
		assert(ConvMesh->NumEdges * 2 == ConvMesh->Edges.size());
		assert(ConvMesh->NumFaces == ConvMesh->Faces.size());

		return Geom;
	}

	static Geometry* CreateShape(const physx::NpShape *shape)
	{
		if (shape == nullptr)
		{
			return nullptr;
		}

		Geometry* Geom = nullptr;
		int Type = shape->getGeomType();
		if (Type == physx::eSPHERE)
		{
			physx::PxSphereGeometry* sphere = (physx::PxSphereGeometry*)shape->mShape.mShape.mCore.geometry.mGeometry.sphere;
			Geom = CreateSphere(sphere);
		}
		else if (Type == physx::ePLANE)
		{
			physx::PxPlaneGeometry* plane = (physx::PxPlaneGeometry*)shape->mShape.mShape.mCore.geometry.mGeometry.plane;
			Geom = CreatePlane(plane);
		}
		else if (Type == physx::eCAPSULE)
		{
			physx::PxCapsuleGeometry* capsule = (physx::PxCapsuleGeometry*)shape->mShape.mShape.mCore.geometry.mGeometry.capsule;
			Geom = CreateCapsule(capsule);
		}
		else if (Type == physx::eBOX)
		{
			physx::PxBoxGeometry* box = (physx::PxBoxGeometry*)shape->mShape.mShape.mCore.geometry.mGeometry.box;
			Geom = CreateBox(box);
		}
		else if (Type == physx::eCONVEXMESH)
		{
			physx::PxConvexMeshGeometry* convex = (physx::PxConvexMeshGeometry*)shape->mShape.mShape.mCore.geometry.mGeometry.convex;
			Geom = CreateConvexMesh(convex);
		}
		else if (Type == physx::eTRIANGLEMESH)
		{
			physx::PxTriangleMeshGeometry* pxMesh = (physx::PxTriangleMeshGeometry*)shape->mShape.mShape.mCore.geometry.mGeometry.mesh;
			Geom = CreateTriangleMesh(pxMesh);
		}
		else if (Type == physx::eHEIGHTFIELD)
		{
			physx::PxHeightFieldGeometry* hightfield = (physx::PxHeightFieldGeometry*)shape->mShape.mShape.mCore.geometry.mGeometry.heightfield;
			Geom = CreateHeightField(hightfield);
		}
		else
		{
			assert(false);
		}

		if (Geom)
		{
			// Geom->SetName(shape->mName);
		}

		return Geom;
	}

	static void CreateGeometryObjects(void *px, int classType, uint64_t guid, std::vector<Geometry*> *objs)
	{
		if (classType == physx::PxConcreteType::eMATERIAL)
		{
			// physx::NpMaterial *material = (physx::NpMaterial*)px;
			return;
		}
		else if (classType == physx::PxConcreteType::eRIGID_STATIC)
		{
			physx::NpRigidStatic* rigid = (physx::NpRigidStatic*)px;
			int nShapes = rigid->GetNumShapes();
			physx::NpShape* const* pShades = rigid->GetShapes();
			for (int i = 0; i < nShapes; ++i)
			{
				Geometry* p = CreateShape(pShades[i]);
				if (p)
				{
					p->SetGuid(guid);
					p->SetPosition(rigid->mRigidStatic.mStatic.mCore.body2World.p);
					p->SetRotationQuat(rigid->mRigidStatic.mStatic.mCore.body2World.q);
					objs->push_back(p);
				}
			}
			
			return;
		}
		else if (classType == physx::PxConcreteType::eRIGID_DYNAMIC)
		{
			physx::NpRigidDynamic* rigid = (physx::NpRigidDynamic*)px;
			const physx::PxsBodyCore& core = rigid->mBody.mBodyCore.mCore;

			int nShapes = rigid->GetNumShapes();
			physx::NpShape* const* pShades = rigid->GetShapes();
			for (int i = 0; i < nShapes; ++i)
			{
				Geometry* p = CreateShape(pShades[i]);
				if (p)
				{
					p->SetGuid(guid);
					p->SetPosition(core.body2World.p);
					p->SetRotationQuat(core.body2World.q);
					objs->push_back(p);
				}
			}

			RigidBodyParam param;
			param.Mass = 1.0f / core.inverseMass;
			param.Inertia = Matrix3d(core.inverseInertia.x, core.inverseInertia.y, core.inverseInertia.z).Inverse();
			param.LinearVelocity = core.linearVelocity;
			param.AngularVelocity = core.angularVelocity;
			param.LinearDamping = core.linearDamping;
			param.AngularDamping = core.angularDamping;
			param.ContactReportThreshold = core.contactReportThreshold;
			param.MaxContactImpulse = core.maxContactImpulse;
			param.SleepThreshold = core.sleepThreshold;
			param.FreezeThreshold = core.freezeThreshold;
			param.DisableGravity = false;
			param.Static = false;
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
	uint64_t filesize = (uint64_t)ftell(fp);
	fseek(fp, 0, 0);
	buffer.resize(filesize + 127);
	void* p = AlignMemory(&buffer[0], 128);
	fread(p, 1, filesize, fp);
	fclose(fp);

	PhysxCollections collection;
	if (!physx::DeserializeFromBuffer(p, collection))
	{
		return false;
	}

	for (auto it : collection.mClass)
	{
		PhysxBinaryParser::CreateGeometryObjects(it.first, it.second, collection.mObjects[it.first], GeometryList);
	}

	return collection.mObjects.size() > 0;
}
