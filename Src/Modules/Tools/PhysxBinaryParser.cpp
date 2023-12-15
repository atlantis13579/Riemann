#include <assert.h>
#include <stdio.h>
#include <cstddef>
#include <unordered_map>
#include <vector>

#ifdef __linux__
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/mman.h>
#endif

#include "PhysxBinaryParser.h"
#include "Serialization.h"
#include "../../Collision/GeometryObject.h"
#include "../../CollisionPrimitive/ConvexMesh.h"
#include "../../CollisionPrimitive/HeightField3d.h"
#include "../../CollisionPrimitive/TriangleMesh.h"
#include "../../CollisionPrimitive/MeshBVH4.h"
#include "../../RigidBodyDynamics/RigidBody.h"
#include "../../Core/Base.h"
#include "../../Maths/Box3d.h"
#include "../../Maths/Transform.h"
#include "../../Maths/Quaternion.h"

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
typedef Vector3 PxVec3;

typedef uint64_t PxSerialObjectId;

struct PhysxCollections
{
	std::unordered_map<PxSerialObjectId, void*>		mIds;
	std::unordered_map<void*, PxSerialObjectId>		mObjects;
	std::unordered_map<void*, int>					mClass;
};

enum BinaryPlatform
{
	PLATFORM_UNKNOWN = 0,
	WIN64 = 1,
	LINUX64 = 2,
};

#include "PhysxFormat_34.hpp"
#define physx	PhysxFormat_34

class PhysxBinaryParser
{
public:
	static Geometry* CreateSphere(physx::PxSphereGeometry* physxObj)
	{
		return GeometryFactory::CreateSphere(Vector3::Zero(), physxObj->radius);
	}

	static Geometry* CreatePlane(physx::PxPlaneGeometry* physxObj)
	{
		return GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY());
	}

	static Geometry* CreateCapsule(physx::PxCapsuleGeometry* physxObj)
	{
		return GeometryFactory::CreateCapsule(Vector3::UnitY() * -physxObj->halfHeight, Vector3::UnitY() * physxObj->halfHeight, physxObj->radius);
	}

	static Geometry* CreateBox(physx::PxBoxGeometry* physxObj)
	{
		return GeometryFactory::CreateOBB(Vector3::Zero(), physxObj->halfExtents);
	}

	static Geometry* CreateTriangleMesh(const physx::PxTriangleMeshGeometry* physxObj, bool shared_mem)
	{
		const physx::PxRTreeTriangleMesh* Mesh = (const physx::PxRTreeTriangleMesh*)physxObj->triangleMesh;

		Geometry *Geom = GeometryFactory::CreateTriangleMesh();
		TriangleMesh* TriMesh = Geom->GetShapeObj<TriangleMesh>();
		TriMesh->SetData(Mesh->mVertices, Mesh->mTriangles, Mesh->mNbVertices, Mesh->mNbTriangles, Mesh->Is16BitIndices(), !shared_mem);
		TriMesh->BoundingVolume = Mesh->mAABB.GetAABB();

		MeshBVH4* tree = TriMesh->CreateEmptyBVH();
		static_assert(sizeof(MeshBVH4) == sizeof(physx::RTree), "MeshBVH and RTree should have same size");
		static_assert(sizeof(BVHNodeBatch) == sizeof(physx::RTreePage), "BVHNodeBatch and RTreePage should have same size");
		memcpy(tree, &Mesh->mRTree, sizeof(MeshBVH4));
		tree->Memory = nullptr;
		if (shared_mem)
		{
			tree->BatchPtr = (BVHNodeBatch*)Mesh->mRTree.mPages;
		}
		else
		{
			void* pMem = tree->AllocMemory(Mesh->mRTree.mTotalPages * sizeof(physx::RTreePage), 128);
			memcpy(pMem, Mesh->mRTree.mPages, Mesh->mRTree.mTotalPages * sizeof(physx::RTreePage));
		}
		return Geom;
	}

	static Geometry* CreateHeightField(const physx::PxHeightFieldGeometry* physxObj, bool shared_mem)
	{
		const physx::HeightField* pxhf = physxObj->heightField;
		physx::PxHeightFieldSample* samples = pxhf->mData.samples;
		const uint32_t					nCols = pxhf->mData.columns;
		const uint32_t					nRows = pxhf->mData.rows;

		TCE3<float>	ce = pxhf->mData.mAABB;
		Vector3 Scale = Vector3(physxObj->rowScale, physxObj->heightScale, physxObj->columnScale);
		ce.Center *= Scale;
		ce.Extent *= Scale;
		Geometry* Geom = GeometryFactory::CreateHeightField(ce.GetAABB(), nRows, nCols);
		HeightField3d* HF = Geom->GetShapeObj<HeightField3d>();

		if (shared_mem)
		{
			HF->Cells = (HeightField3d::CellInfo*)samples;
			HF->HeightScale = physxObj->heightScale;
		}
		else
		{
			HF->AllocMemory();
			HF->HeightScale = physxObj->heightScale;
			for (uint32_t i = 0; i < nRows; i++)
			for (uint32_t j = 0; j < nCols; j++)
			{
				uint8_t *cell = (uint8_t*)(HF->Cells + (i * nCols + j));
				int16_t *height = (int16_t*)cell;
				uint8_t	*Tess = cell + 2;
				
				height[0] = samples[i * nCols + j].height;
				Tess[0] = samples[i * nCols + j].materialIndex0;
				Tess[1] = samples[i * nCols + j].materialIndex1;
			}
		}

		return Geom;
	}

	static Geometry* CreateConvexMesh(const physx::PxConvexMeshGeometry* physxObj, bool shared_mem)
	{
		physx::GuConvexMesh* Mesh = physxObj->convexMesh;

		Geometry* Geom = GeometryFactory::CreateConvexMesh();
		ConvexMesh* ConvMesh = Geom->GetShapeObj<ConvexMesh>();
		physx::ConvexHullData& hull = Mesh->mHullData;
		
		assert(hull.getHullVertices());
		assert(hull.getVerticesByEdges16());
		assert(hull.getVertexData8());
		
		PxU16 maxIndices = 0;
		for (int i = 0; i < hull.mNbPolygons; ++i)
		{
			const physx::GuHullPolygonData& poly = hull.mPolygons[i];
			maxIndices = std::max(maxIndices, PxU16(poly.mMinIndex + poly.mNbVerts));
		}
		
		ConvMesh->SetConvexData((Vector3*)hull.getHullVertices(), hull.mNbHullVertices,
								(ConvexMeshFace*)hull.mPolygons, hull.mNbPolygons,
								hull.getVerticesByEdges16(), hull.mNbEdges & ~0x8000,
								hull.getVertexData8(), maxIndices,
								shared_mem);
		assert(ConvMesh->ValidateStructure());

		MassParameters* vp = Geom->GetMassParameters();
		vp->Mass = Mesh->mMass;
		vp->Volume = Mesh->mMass;
		vp->InertiaMat = Mesh->mInertia;
		vp->CenterOfMass = hull.mCenterOfMass;
		vp->BoundingVolume = hull.mAABB.GetAABB();
		return Geom;
	}

	static Geometry* CreateShape(const physx::NpShape *shape, bool shared_mem)
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
			Geom = CreateConvexMesh(convex, shared_mem);
		}
		else if (Type == physx::eTRIANGLEMESH)
		{
			physx::PxTriangleMeshGeometry* pxMesh = (physx::PxTriangleMeshGeometry*)shape->mShape.mShape.mCore.geometry.mGeometry.mesh;
			Geom = CreateTriangleMesh(pxMesh, shared_mem);
		}
		else if (Type == physx::eHEIGHTFIELD)
		{
			physx::PxHeightFieldGeometry* hightfield = (physx::PxHeightFieldGeometry*)shape->mShape.mShape.mCore.geometry.mGeometry.heightfield;
			Geom = CreateHeightField(hightfield, shared_mem);
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

	static void CreateRigidBodies(void *px, int classType, uint64_t guid, std::vector<RigidBody*> *bodies, std::vector<Geometry*>* geoms, bool shared_mem)
	{
		if (classType == physx::PxConcreteType::eMATERIAL)
		{
			physx::NpMaterial *material = (physx::NpMaterial*)px;
            UNUSED(material);
			// TODO
			return;
		}
		else if (classType == physx::PxConcreteType::eRIGID_STATIC)
		{
			physx::NpRigidStatic* rigid = (physx::NpRigidStatic*)px;

			RigidBodyStatic* body = nullptr;
			if (bodies)
			{
				RigidBodyParam param;
				Pose init_pose(rigid->mRigidStatic.mStatic.mCore.body2World.p, rigid->mRigidStatic.mStatic.mCore.body2World.q);
				body = RigidBodyStatic::CreateRigidBody(param, init_pose);
				assert(body);
				body->SetGuid(guid);
				bodies->push_back(body);
			}

			int nShapes = rigid->GetNumShapes();
			physx::NpShape* const* pShades = rigid->GetShapes();
			for (int i = 0; i < nShapes; ++i)
			{
				Geometry* g = CreateShape(pShades[i], shared_mem);
				if (g)
				{
					g->SetWorldTransform(rigid->mRigidStatic.mStatic.mCore.body2World.p, rigid->mRigidStatic.mStatic.mCore.body2World.q);
					if (geoms) geoms->push_back(g);
					if (bodies) body->AddGeometry(g);
				}
			}

			return;
		}
		else if (classType == physx::PxConcreteType::eRIGID_DYNAMIC)
		{
			physx::NpRigidDynamic* rigid = (physx::NpRigidDynamic*)px;
			const physx::PxsBodyCore& core = rigid->mBody.mBodyCore.mCore;

			RigidBodyDynamic* body = nullptr;
			if (bodies)
			{
				RigidBodyParam param;
				param.invMass = core.inverseMass;
				param.inertia = Matrix3(core.inverseInertia.x, core.inverseInertia.y, core.inverseInertia.z).Inverse();
				param.linearVelocity = core.linearVelocity;
				param.angularVelocity = core.angularVelocity;
				param.linearDamping = core.linearDamping;
				param.angularDamping = core.angularDamping;
				param.contactReportThreshold = core.contactReportThreshold;
				param.maxContactImpulse = core.maxContactImpulse;
				param.sleepThreshold = core.sleepThreshold;
				param.freezeThreshold = core.freezeThreshold;
				param.disableGravity = false;

				Pose init_pose(core.body2World.p, core.body2World.q);

				body = RigidBodyDynamic::CreateRigidBody(param, init_pose);
				body->SetGuid(guid);
				bodies->push_back(body);
			}

			int nShapes = rigid->GetNumShapes();
			physx::NpShape* const* pShades = rigid->GetShapes();
			for (int i = 0; i < nShapes; ++i)
			{
				Geometry* g = CreateShape(pShades[i], shared_mem);
				if (g)
				{
					g->SetWorldTransform(core.body2World.p, core.body2World.q);
					if (geoms) geoms->push_back(g);
					if (bodies) body->AddGeometry(g);
				}
			}

			return;
		}
		else
		{
			// TODO
			return;
		}
		return;
	}

	static void GenerateShapeTriangles(const physx::NpShape* shape, std::vector<Vector3>& vertices, std::vector<int>& indices, const Vector3& p, const Quaternion& q)
	{
		if (shape == nullptr)
		{
			return;
		}

		int Type = shape->getGeomType();
		if (Type == physx::eCONVEXMESH)
		{
			physx::PxConvexMeshGeometry* convex = (physx::PxConvexMeshGeometry*)shape->mShape.mShape.mCore.geometry.mGeometry.convex;
			physx::ConvexHullData& hull = convex->convexMesh->mHullData;

			Vector3 *v = (Vector3*)hull.getHullVertices();
			PxU8* ind = hull.getVertexData8();

			for (PxU8 i = 0; i < hull.mNbHullVertices; ++i)
			{
				vertices.push_back(q * v[i] + p);
			}

			int indices_begin = (int)vertices.size();
			for (PxU8 i = 0; i < hull.mNbPolygons; ++i)
			{
				const physx::GuHullPolygonData& poly = hull.mPolygons[i];
				for (PxU8 j = 1; j < poly.mNbVerts - 1; ++j)
				{
					indices.push_back(indices_begin + ind[poly.mVRef8]);
					indices.push_back(indices_begin + ind[poly.mVRef8 + j]);
					indices.push_back(indices_begin + ind[poly.mVRef8 + j + 1]);
				}
			}
		}
		else if (Type == physx::eTRIANGLEMESH)
		{
			int indices_begin = (int)vertices.size();

			physx::PxTriangleMeshGeometry* pxMesh = (physx::PxTriangleMeshGeometry*)shape->mShape.mShape.mCore.geometry.mGeometry.mesh;
			const physx::PxRTreeTriangleMesh* Mesh = (const physx::PxRTreeTriangleMesh*)pxMesh->triangleMesh;

			for (PxU32 i = 0; i < Mesh->mNbVertices; ++i)
			{
				vertices.push_back(q * Mesh->mVertices[i] + p);
			}

			if (Mesh->Is16BitIndices())
			{
				uint16_t* indices16 = (uint16_t*)Mesh->mTriangles;
				for (PxU32 i = 0; i < Mesh->mNbTriangles; ++i)
				{
					uint16_t a = indices16[3 * i + 0];
					uint16_t b = indices16[3 * i + 1];
					uint16_t c = indices16[3 * i + 2];
					indices.push_back(indices_begin + (int)a);
					indices.push_back(indices_begin + (int)b);
					indices.push_back(indices_begin + (int)c);
				}
			}
			else
			{
				uint32_t* indices32 = (uint32_t*)Mesh->mTriangles;
				for (PxU32 i = 0; i < Mesh->mNbTriangles; ++i)
				{
					uint32_t a = indices32[3 * i + 0];
					uint32_t b = indices32[3 * i + 1];
					uint32_t c = indices32[3 * i + 2];
					indices.push_back(indices_begin + (int)a);
					indices.push_back(indices_begin + (int)b);
					indices.push_back(indices_begin + (int)c);
				}
			}
		}
		else if (Type == physx::eHEIGHTFIELD)
		{
			physx::PxHeightFieldGeometry* physxObj = (physx::PxHeightFieldGeometry*)shape->mShape.mShape.mCore.geometry.mGeometry.heightfield;
			const physx::HeightField* pxhf = physxObj->heightField;
			physx::PxHeightFieldSample* samples = pxhf->mData.samples;
			const uint32_t					nCols = pxhf->mData.columns;
			const uint32_t					nRows = pxhf->mData.rows;

			TCE3<float>	ce = pxhf->mData.mAABB;
			Vector3 Scale = Vector3(physxObj->rowScale, physxObj->heightScale, physxObj->columnScale);
			ce.Center *= Scale;
			ce.Extent *= Scale;
			TAABB3<float> BV = ce.GetAABB();
			float DX = (BV.mMax.x - BV.mMin.x) / (nRows - 1);
			float DZ = (BV.mMax.z - BV.mMin.z) / (nCols - 1);

			int indices_begin = (int)vertices.size();

			for (PxU32 i = 0; i < nRows; i++)
			for (PxU32 j = 0; j < nCols; j++)
			{
				Vector3 v = Vector3(BV.mMin.x + DX * i, samples[i * nCols + j].height * Scale.y, BV.mMin.z + DZ * j);
				vertices.push_back(q * v + p);
			}

			for (PxU32 i = 0; i < nRows - 1; i++)
			for (PxU32 j = 0; j < nCols - 1; j++)
			{
				bool tessFlag = samples[j + i * nCols].materialIndex0 & 0x80;
				uint16_t i0 = i * nCols + j;
				uint16_t i1 = i * nCols + j + 1;
				uint16_t i2 = (i + 1) * nCols + j;
				uint16_t i3 = (i + 1) * nCols + j + 1;
				// i2---i3
				// |    |
				// |    |
				// i0---i1
				uint8_t Hole0 = samples[j + i * nCols].materialIndex0;
				uint8_t Hole1 = samples[j + i * nCols].materialIndex1;
				
				if (Hole0 != 0x7F)
				{
					indices.push_back(indices_begin + i2);
					indices.push_back(indices_begin + i0);
					indices.push_back(indices_begin + (tessFlag ? i3 : i1));
				}
				if (Hole1 != 0x7F)
				{
					indices.push_back(indices_begin + i3);
					indices.push_back(indices_begin + (tessFlag ? i0 : i2));
					indices.push_back(indices_begin + i1);
				}
			}
		}
		else if (Type == physx::eBOX)
		{
			physx::PxBoxGeometry* box = (physx::PxBoxGeometry*)shape->mShape.mShape.mCore.geometry.mGeometry.box;
			const PxVec3& ext = box->halfExtents;
			PxVec3 v[8];
			v[0] = p + q * PxVec3(-ext.x, -ext.y, -ext.z);
			v[1] = p + q * PxVec3(ext.x, -ext.y, -ext.z);
			v[2] = p + q * PxVec3(-ext.x, ext.y, -ext.z);
			v[3] = p + q * PxVec3(ext.x, ext.y, -ext.z);
			v[4] = p + q * PxVec3(-ext.x, -ext.y, ext.z);
			v[5] = p + q * PxVec3(ext.x, -ext.y, ext.z);
			v[6] = p + q * PxVec3(-ext.x, ext.y, ext.z);
			v[7] = p + q * PxVec3(ext.x, ext.y, ext.z);
			for (int j = 0; j < 8; j++)
			{
				vertices.push_back(v[j]);
			}
			const int ind[] = {
				0, 1, 2,
				1, 3, 2,
				4, 5, 6,
				5, 7, 6,
				0, 1, 4,
				5, 4, 1,
				1, 3, 5,
				7, 5, 3,
				2, 4, 0,
				6, 4, 2,
				3, 2, 6,
				6, 7, 3
			};
			for (int j = 0; j < sizeof(ind) / sizeof(ind[0]); j++)
			{
				indices.push_back(ind[j]);
			}
		}
		else if (Type == physx::eSPHERE)
		{
			physx::PxSphereGeometry* sphere = (physx::PxSphereGeometry*)shape->mShape.mShape.mCore.geometry.mGeometry.sphere;

			const float mPI = 2.0f * asinf(1.0f);
			const int stackCount = 6;
			const int sliceCount = 8;
			const float phiStep = mPI / stackCount;
			const float thetaStep = 2.0f * mPI / sliceCount;
			const float radius = sphere->radius;

			vertices.push_back(q * Vector3(0, radius, 0) + p);
			for (int i = 1; i < stackCount; i++)
			{
				float phi = i * phiStep;
				for (int j = 0; j <= sliceCount; j++)
				{
					float theta = j * thetaStep;
					Vector3 v = Vector3(sinf(phi) * cosf(theta), cosf(phi), sinf(phi) * sinf(theta)) * radius;
					vertices.push_back(q * v + p);
				}
			}
			vertices.push_back(q * Vector3(0, -radius, 0) + p);

			for (int i = 1; i <= sliceCount; i++)
			{
				indices.push_back(0);
				indices.push_back(i + 1);
				indices.push_back(i);
			}

			int baseIndex = 1;
			int Count = sliceCount + 1;
			for (int i = 0; i < stackCount - 2; i++)
			{
				for (int j = 0; j < sliceCount; j++)
				{
					indices.push_back(baseIndex + i * Count + j);
					indices.push_back(baseIndex + i * Count + j + 1);
					indices.push_back(baseIndex + (i + 1) * Count + j);

					indices.push_back(baseIndex + (i + 1) * Count + j);
					indices.push_back(baseIndex + i * Count + j + 1);
					indices.push_back(baseIndex + (i + 1) * Count + j + 1);
				}
			}
			int PoleIndex = (stackCount - 1) * (sliceCount + 1) + 1;
			baseIndex = PoleIndex - Count;
			for (int i = 0; i < sliceCount; i++)
			{
				indices.push_back(PoleIndex);
				indices.push_back(baseIndex + i);
				indices.push_back(baseIndex + i + 1);
			}
		}
		else
		{
			//	assert(false);
		}
	}

	static void GenerateTriangles(void* px, int classType, uint64_t guid, std::vector<Vector3>& vertices, std::vector<int>& indices)
	{
		if (classType == physx::PxConcreteType::eMATERIAL)
		{
			physx::NpMaterial* material = (physx::NpMaterial*)px;
			UNUSED(material);
			// TODO
			return;
		}
		else if (classType == physx::PxConcreteType::eRIGID_STATIC)
		{
			physx::NpRigidStatic* rigid = (physx::NpRigidStatic*)px;

			int nShapes = rigid->GetNumShapes();
			physx::NpShape* const* pShades = rigid->GetShapes();
			for (int i = 0; i < nShapes; ++i)
			{
				GenerateShapeTriangles(pShades[i], vertices, indices, rigid->mRigidStatic.mStatic.mCore.body2World.p, rigid->mRigidStatic.mStatic.mCore.body2World.q);
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
				GenerateShapeTriangles(pShades[i], vertices, indices, core.body2World.p, core.body2World.q);
			}

			return;
		}
		else
		{
			// TODO
			return;
		}
		return;
	}
};

using namespace physx;

bool DeserializeFromBuffer(void* Buffer, PhysxCollections& collection)
{
	if (size_t(Buffer) & (128 - 1))
	{
		printf("Buffer must be 128-bytes aligned.\n");
		return false;
	}

	uint8_t* address = reinterpret_cast<uint8_t*>(Buffer);

	PxU32 version;
	BinaryPlatform platform;
	if (!readHeader(address, version, platform))
	{
		return false;
	}

	ManifestEntry* manifestTable;
	uint32_t nbObjectsInCollection;
	uint32_t objectDataEndOffset;

	// read number of objects in collection
	address = alignPtr(address);
	nbObjectsInCollection = read32(address);

	// read manifest (uint32_t offset, PxConcreteType type)
	{
		address = alignPtr(address);
		uint32_t nbManifestEntries = read32(address);
		assert(*reinterpret_cast<uint32_t*>(address) == 0); //first offset is always 0
		manifestTable = (nbManifestEntries > 0) ? reinterpret_cast<ManifestEntry*>(address) : NULL;
		address += nbManifestEntries * sizeof(ManifestEntry);
		objectDataEndOffset = read32(address);
	}

	ImportReference* importReferences;
	uint32_t nbImportReferences;
	// read import references
	{
		address = alignPtr(address);
		nbImportReferences = read32(address);
		importReferences = (nbImportReferences > 0) ? reinterpret_cast<ImportReference*>(address) : NULL;
		address += nbImportReferences * sizeof(ImportReference);
	}

	// TODO
	assert(importReferences == nullptr);

	ExportReference* exportReferences;
	uint32_t nbExportReferences;
	// read export references
	{
		address = alignPtr(address);
		nbExportReferences = read32(address);
		exportReferences = (nbExportReferences > 0) ? reinterpret_cast<ExportReference*>(address) : NULL;
		address += nbExportReferences * sizeof(ExportReference);
	}

	// read internal references arrays
	uint32_t nbInternalPtrReferences = 0;
	uint32_t nbInternalHandle16References = 0;
	InternalReferencePtr* internalPtrReferences = NULL;
	InternalReferenceHandle16* internalHandle16References = NULL;
	{
		address = alignPtr(address);

		nbInternalPtrReferences = read32(address);
		internalPtrReferences = (nbInternalPtrReferences > 0) ? reinterpret_cast<InternalReferencePtr*>(address) : NULL;
		address += nbInternalPtrReferences * sizeof(InternalReferencePtr);

		nbInternalHandle16References = read32(address);
		internalHandle16References = (nbInternalHandle16References > 0) ? reinterpret_cast<InternalReferenceHandle16*>(address) : NULL;
		address += nbInternalHandle16References * sizeof(InternalReferenceHandle16);
	}

	std::unordered_map<size_t, SerialObjectIndex> internalPtrReferencesMap(nbInternalPtrReferences * 2);
	{
		//create hash (we should load the hashes directly from memory)
		for (uint32_t i = 0; i < nbInternalPtrReferences; i++)
		{
			const InternalReferencePtr& ref = internalPtrReferences[i];
			internalPtrReferencesMap.emplace(ref.reference, SerialObjectIndex(ref.objIndex));
		}
	}

	std::unordered_map<uint16_t, SerialObjectIndex> internalHandle16ReferencesMap(nbInternalHandle16References * 2);
	{
		for (uint32_t i = 0; i < nbInternalHandle16References; i++)
		{
			const InternalReferenceHandle16& ref = internalHandle16References[i];
			internalHandle16ReferencesMap.emplace(ref.reference, SerialObjectIndex(ref.objIndex));
		}
	}

	collection.mObjects.reserve(nbObjectsInCollection * 2);
	if (nbExportReferences > 0)
		collection.mIds.reserve(nbExportReferences * 2);

	uint8_t* addressObjectData = alignPtr(address);
	uint8_t* addressExtraData = alignPtr(addressObjectData + objectDataEndOffset);
	std::unordered_map<PxType, int> Statistic;

	PxDeserializationContext context(manifestTable, importReferences, addressObjectData, internalPtrReferencesMap, internalHandle16ReferencesMap, addressExtraData, version, platform);

	// iterate over memory containing PxBase objects, create the instances, resolve the addresses, import the external data, add to collection.
	{
		uint32_t nbObjects = nbObjectsInCollection;
		uint8_t* addressBase = address;
		uint8_t* addressExtraDataBase = addressExtraData;

		while (nbObjects--)
		{
			address = alignPtr(address);
			context.alignExtraData();

			// read PxBase header with type and get corresponding serializer.
			PxBase* header = reinterpret_cast<PxBase*>(address);
			const PxType classType = header->getConcreteType();

			// printf("%d\t%d\t%d\n", nbObjects, (int)(address - addressBase), (int)(context.mExtraDataAddress - addressExtraDataBase));
			PxBase* instance = Deserialize(address, context, classType);

			if (Statistic.find(classType) == Statistic.end())
			{
				Statistic.emplace(classType, 0);
			}
			if (!instance)
			{
				Statistic[classType] -= 1;
				continue;
			}
			Statistic[classType] += 1;

			collection.mObjects.emplace(instance, 0);
			collection.mClass.emplace(instance, classType);
		}

		UNUSED(addressBase);
		UNUSED(addressExtraDataBase);
	}

	bool success = true;
	for (auto it : Statistic)
	{
		if (it.second >= 0)
		{
			// printf("Create class instance for type %d success (%d objs).\n", it.first, it.second);
		}
		else
		{
			success = false;
			printf("Create class instance for type %d failed (%d objs).\n", it.first, -it.second);
		}
	}

	assert(collection.mObjects.size() == nbObjectsInCollection);

	// update new collection with export references
	{
		assert(addressObjectData != NULL);
		for (uint32_t i = 0; i < nbExportReferences; i++)
		{
			bool isExternal;
			uint32_t manifestIndex = exportReferences[i].objIndex.getIndex(isExternal);
			assert(!isExternal);
			PxBase* obj = reinterpret_cast<PxBase*>(addressObjectData + manifestTable[manifestIndex].offset);
			collection.mIds.emplace(exportReferences[i].id, obj);
			collection.mObjects[obj] = exportReferences[i].id;
		}
	}
	return success;
}

void	ReleaseSharedMem(void* addr, size_t size)
{
	if (addr == nullptr)
		return;

	#if defined(__linux__)
	munmap(addr, size);
	#else
	delete [](char*)addr;
	#endif
}

bool	LoadPhysxBinary(const char* Filename, std::vector<RigidBody*>* bodies, std::vector<Geometry*>* geoms)
{
	if (bodies) bodies->clear();
	if (geoms) geoms->clear();

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
	if (!DeserializeFromBuffer(p, collection))
	{
		return false;
	}

	for (auto it : collection.mClass)
	{
		PhysxBinaryParser::CreateRigidBodies(it.first, it.second, collection.mObjects[it.first], bodies, geoms, false);
	}

	return collection.mObjects.size() > 0;
}

void*	LoadPhysxBinaryMmap(const char* Filename, std::vector<RigidBody*>* bodies, std::vector<Geometry*>* geoms, size_t &mem_size)
{
	if (bodies) bodies->clear();
	if (geoms) geoms->clear();

#if defined(__linux__)
	int fd = open(Filename, O_RDONLY);
	if (fd == -1)
	{
		return nullptr;
	}

	struct stat st;
	fstat(fd, &st);
	mem_size = st.st_size;
	void* addr = mmap(nullptr, mem_size, PROT_READ | PROT_WRITE, MAP_PRIVATE, fd, 0);
	if (addr == MAP_FAILED)
	{
		close(fd);
		return nullptr;
	}
	void* p128 = AlignMemory(addr, 128);
	assert(p128 == addr);
#else
	FILE* fp = fopen(Filename, "rb");
	if (fp == nullptr)
	{
		return nullptr;
	}
	fseek(fp, 0, SEEK_END);
	size_t bytes = (size_t)ftell(fp);
	fseek(fp, 0, 0);
	void *addr = new char[bytes + 127];
	mem_size = bytes + 127;
	void* p128 = AlignMemory(addr, 128);
	fread(p128, 1, bytes, fp);
	fclose(fp);
#endif

	PhysxCollections collection;
	if (!DeserializeFromBuffer(p128, collection))
	{
		#if defined(__linux__)
		close(fd);
		#endif
		ReleaseSharedMem(addr, mem_size);
		return nullptr;
	}

	for (auto it : collection.mClass)
	{
		PhysxBinaryParser::CreateRigidBodies(it.first, it.second, collection.mObjects[it.first], bodies, geoms, true);
	}

	return addr;
}

bool	LoadPhysxBinaryTriangles(const char* Filename, std::vector<Vector3>& vertices, std::vector<int>& indices)
{
	vertices.clear();
	indices.clear();

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
	if (!DeserializeFromBuffer(p, collection))
	{
		return false;
	}

	for (auto it : collection.mClass)
	{
		PhysxBinaryParser::GenerateTriangles(it.first, it.second, collection.mObjects[it.first], vertices, indices);
	}

	return collection.mObjects.size() > 0;
}
