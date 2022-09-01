#pragma once

#include <cstddef>
#include <unordered_map>
#include <vector>

namespace PhysxFormat_41
{
	struct PxConcreteType
	{
		enum
		{
			eUNDEFINED,

			e_HEIGHTFIELD,
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
	};

	typedef uint16_t PxType;

	struct ManifestEntry
	{
		ManifestEntry(uint32_t _offset, PxType _type)
		{
			// Cm::markSerializedMem(this, sizeof(ManifestEntry));
			offset = _offset;
			type = _type;
		}
		ManifestEntry()
		{
			// Cm::markSerializedMem(this, sizeof(ManifestEntry));
		}
		void operator =(const ManifestEntry& m)
		{
			memcpy(this, &m, sizeof(ManifestEntry));
		}

		uint32_t offset;
		PxType type;
	};

	struct ImportReference
	{
		ImportReference(PxSerialObjectId _id, PxType _type)
		{
			// Cm::markSerializedMem(this, sizeof(ImportReference));
			id = _id;
			type = _type;
		}
		ImportReference()
		{
			// Cm::markSerializedMem(this, sizeof(ImportReference));
		}
		void operator =(const ImportReference& m)
		{
			memcpy(this, &m, sizeof(ImportReference));
		}
		PxSerialObjectId id;
		PxType type;
	};

#define SERIAL_OBJECT_INDEX_TYPE_BIT (1u<<31)
	struct SerialObjectIndex
	{
		SerialObjectIndex(uint32_t index, bool external) { setIndex(index, external); }
		SerialObjectIndex(const SerialObjectIndex& objIndex) : mObjIndex(objIndex.mObjIndex) {}
		SerialObjectIndex() : mObjIndex(0xFFFFFFFF) {}

		void setIndex(uint32_t index, bool external)
		{
			assert((index & SERIAL_OBJECT_INDEX_TYPE_BIT) == 0);
			mObjIndex = index | (external ? SERIAL_OBJECT_INDEX_TYPE_BIT : 0);
		}

		uint32_t getIndex(bool& isExternal)
		{
			assert(mObjIndex != 0xFFFFFFFF);
			isExternal = (mObjIndex & SERIAL_OBJECT_INDEX_TYPE_BIT) > 0;
			return mObjIndex & ~SERIAL_OBJECT_INDEX_TYPE_BIT;
		}

		bool operator < (const SerialObjectIndex& so) const
		{
			return mObjIndex < so.mObjIndex;
		}

	private:
		uint32_t mObjIndex;
	};

	struct ExportReference
	{
		ExportReference(PxSerialObjectId _id, SerialObjectIndex _objIndex)
		{
			// Cm::markSerializedMem(this, sizeof(ExportReference));
			id = _id;
			objIndex = _objIndex;
		}
		ExportReference()
		{
			// Cm::markSerializedMem(this, sizeof(ExportReference));
		}
		void operator =(const ExportReference& m)
		{
			memcpy(this, &m, sizeof(ExportReference));
		}
		PxSerialObjectId id;
		SerialObjectIndex objIndex;
	};

	struct InternalReferencePtr
	{
		InternalReferencePtr() {}

		InternalReferencePtr(size_t _reference, SerialObjectIndex _objIndex) :
			reference(_reference),
			objIndex(_objIndex)
#if defined(__x86_64__) || defined(__arm64__) || defined(__aarch64__)
			, pad(0xcdcdcdcd)
#endif
		{
		}

		size_t reference;
		SerialObjectIndex objIndex;
#if defined(__x86_64__) || defined(__arm64__) || defined(__aarch64__)
		uint32_t pad;
#endif
	};

	struct InternalReferenceHandle16
	{
		InternalReferenceHandle16() {}

		InternalReferenceHandle16(uint16_t _reference, SerialObjectIndex _objIndex) :
			reference(_reference),
			pad(0xcdcd),
			objIndex(_objIndex)
		{
		}

		uint16_t reference;
		uint16_t pad;
		SerialObjectIndex objIndex;
	};

	template<typename T> struct PxTypeInfo {};

#define PX_DEFINE_TYPEINFO(_name, _fastType) \
	class _name; \
	template <> struct PxTypeInfo<_name>	{	static const char* name() { return #_name;	}	enum { eFastTypeId = _fastType };	};

	PX_DEFINE_TYPEINFO(PxCPxBase, PxConcreteType::eUNDEFINED)
	PX_DEFINE_TYPEINFO(PxMaterial, PxConcreteType::eMATERIAL)
	PX_DEFINE_TYPEINFO(ConvexMesh, PxConcreteType::eCONVEX_MESH)
	PX_DEFINE_TYPEINFO(PxTriangleMesh, PxConcreteType::eUNDEFINED)
	PX_DEFINE_TYPEINFO(PxBVH33TriangleMesh, PxConcreteType::eTRIANGLE_MESH_BVH33)
	PX_DEFINE_TYPEINFO(PxBVH34TriangleMesh, PxConcreteType::eTRIANGLE_MESH_BVH34)
	PX_DEFINE_TYPEINFO(HeightField, PxConcreteType::e_HEIGHTFIELD)
	PX_DEFINE_TYPEINFO(PxActor, PxConcreteType::eUNDEFINED)
	PX_DEFINE_TYPEINFO(PxRigidActor, PxConcreteType::eUNDEFINED)
	PX_DEFINE_TYPEINFO(PxRigidBody, PxConcreteType::eUNDEFINED)
	PX_DEFINE_TYPEINFO(PxRigidDynamic, PxConcreteType::eRIGID_DYNAMIC)
	PX_DEFINE_TYPEINFO(PxRigidStatic, PxConcreteType::eRIGID_STATIC)
	PX_DEFINE_TYPEINFO(PxArticulationLink, PxConcreteType::eARTICULATION_LINK)
	PX_DEFINE_TYPEINFO(PxArticulationJoint, PxConcreteType::eARTICULATION_JOINT)
	PX_DEFINE_TYPEINFO(PxArticulationJointReducedCoordinate, PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE)
	PX_DEFINE_TYPEINFO(PxArticulation, PxConcreteType::eARTICULATION)
	PX_DEFINE_TYPEINFO(PxArticulationReducedCoordinate, PxConcreteType::eARTICULATION_REDUCED_COORDINATE)
	PX_DEFINE_TYPEINFO(PxAggregate, PxConcreteType::eAGGREGATE)
	PX_DEFINE_TYPEINFO(PxConstraint, PxConcreteType::eCONSTRAINT)
	PX_DEFINE_TYPEINFO(PxShape, PxConcreteType::eSHAPE)
	PX_DEFINE_TYPEINFO(PxPruningStructure, PxConcreteType::ePRUNING_STRUCTURE)

	class PxBase
	{
	public:
		virtual				~PxBase() {};
		virtual				bool		isKindOf(const char* superClass) const { return !::strcmp(superClass, "PxBase"); }

		PxType				getConcreteType() const { return (PxType)mConcreteType; }
		template<class T> T* is() { return typeMatch<T>() ? static_cast<T*>(this) : NULL; }
		template<class T>	bool	typeMatch() const
		{
			return PxU32(PxTypeInfo<T>::eFastTypeId) != PxU32(PxConcreteType::eUNDEFINED) ?
				PxU32(getConcreteType()) == PxU32(PxTypeInfo<T>::eFastTypeId) : isKindOf(PxTypeInfo<T>::name());
		}

		uint16_t		mConcreteType;			// concrete type identifier - see PxConcreteType.
		uint16_t		mBaseFlags;				// internal flags
	};

	class PxActor : public PxBase
	{
	public:
		void* UserData;
	};

	class PxRigidActor : public PxActor
	{
	public:
		virtual						~PxRigidActor() {}
		virtual		bool			isKindOf(const char* name)	const { return !::strcmp("PxRigidActor", name) || PxActor::isKindOf(name); }
	};

	class PxRigidBody : public PxRigidActor
	{
	public:
		virtual						~PxRigidBody() {}
		virtual		bool			isKindOf(const char* name)const { return !::strcmp("PxRigidBody", name) || PxRigidActor::isKindOf(name); }
	};

	class PxRigidStatic : public PxRigidActor
	{
	public:
		virtual						~PxRigidStatic() {}
		virtual		bool			isKindOf(const char* name)	const { return !::strcmp("PxRigidStatic", name) || PxRigidActor::isKindOf(name); }
	};

	class PxRigidDynamic : public PxRigidBody
	{
	public:
		virtual							~PxRigidDynamic() {}
		virtual		bool				isKindOf(const char* name) const { return !::strcmp("PxRigidDynamic", name) || PxRigidBody::isKindOf(name); }
	};

	class PxDeserializationContext
	{
	public:
		PxDeserializationContext(const ManifestEntry* manifestTable,
			const ImportReference* importReferences,
			uint8_t* objectDataAddress,
			const std::unordered_map<size_t, SerialObjectIndex>& internalPtrReferencesMap,
			const std::unordered_map<uint16_t, SerialObjectIndex>& internalHandle16ReferencesMap,
			// const Cm::Collection* externalRefs,
			uint8_t* extraData,
			PxU32 physxVersion,
			BinaryPlatform platform)
			: mManifestTable(manifestTable)
			, mImportReferences(importReferences)
			, mObjectDataAddress(objectDataAddress)
			, mInternalPtrReferencesMap(internalPtrReferencesMap)
			, mInternalHandle16ReferencesMap(internalHandle16ReferencesMap)
			// , mExternalRefs(externalRefs)
			, mPhysXVersion(physxVersion)
			, mBinaryPlatform(platform)
		{
			mExtraDataAddress = extraData;
		}

		~PxDeserializationContext() {}

		void			readName(const char*& name)
		{
			uint32_t len = *reinterpret_cast<uint32_t*>(mExtraDataAddress);
			mExtraDataAddress += sizeof(len);
			name = len ? reinterpret_cast<const char*>(mExtraDataAddress) : NULL;
			mExtraDataAddress += len;
		}

		template<typename T>
		T* readExtraData(uint32_t count = 1)
		{
			T* data = reinterpret_cast<T*>(mExtraDataAddress);
			mExtraDataAddress += sizeof(T) * count;
			return data;
		}

		template<typename T, uint32_t alignment>
		T* readExtraData(uint32_t count = 1)
		{
			alignExtraData(alignment);
			return readExtraData<T>(count);
		}

		void			alignExtraData(uint32_t alignment = 16)
		{
			size_t addr = reinterpret_cast<size_t>(mExtraDataAddress);
			addr = (addr + alignment - 1) & ~size_t(alignment - 1);
			mExtraDataAddress = reinterpret_cast<uint8_t*>(addr);
		}

#define PX_SERIAL_REF_KIND_PTR_TYPE_BIT (1u<<31)
#define PX_SERIAL_REF_KIND_PXBASE		(0 | PX_SERIAL_REF_KIND_PTR_TYPE_BIT)
#define PX_SERIAL_REF_KIND_MATERIAL_IDX (1)
		PxBase* resolveReference(PxU32 kind, size_t reference) const
		{
			SerialObjectIndex objIndex;
			if (kind == PX_SERIAL_REF_KIND_PXBASE)
			{
				auto entry0 = mInternalPtrReferencesMap.find(reference);
				assert(entry0 != mInternalPtrReferencesMap.end());
				objIndex = entry0->second;
			}
			else if (kind == PX_SERIAL_REF_KIND_MATERIAL_IDX)
			{
				auto entry0 = mInternalHandle16ReferencesMap.find(PxU16(reference));
				assert(entry0 != mInternalHandle16ReferencesMap.end());
				objIndex = entry0->second;
			}
			else
			{
				return NULL;
			}

			bool isExternal;
			PxU32 index = objIndex.getIndex(isExternal);
			PxBase* base = NULL;
			if (isExternal)
			{
				assert(false);
				const ImportReference& entry = mImportReferences[index];
				// base = mExternalRefs->find(entry.id);
			}
			else
			{
				const ManifestEntry& entry = mManifestTable[index];
				base = reinterpret_cast<PxBase*>(mObjectDataAddress + entry.offset);
			}
			assert(base);
			return base;
		}

		template<typename T>
		void			translatePxBase(T*& base) { if (base) { base = static_cast<T*>(resolveReference(PX_SERIAL_REF_KIND_PXBASE, size_t(base))); } }

	public:
		uint8_t* mExtraDataAddress;

		const ManifestEntry* mManifestTable;
		const ImportReference* mImportReferences;
		uint8_t* mObjectDataAddress;

		//internal references maps for resolving references.
		const std::unordered_map<size_t, SerialObjectIndex>& mInternalPtrReferencesMap;
		const std::unordered_map<uint16_t, SerialObjectIndex>& mInternalHandle16ReferencesMap;

		//external collection for resolving import references.
		// const Cm::Collection* mExternalRefs;

		const PxU32 mPhysXVersion;
		const BinaryPlatform mBinaryPlatform;
	};

	class PxRefCountable
	{
	public:
		virtual ~PxRefCountable() {}
		int mRefCount;
	};

	class PxTriangleMesh : public PxBase, public PxRefCountable
	{
	public:
		virtual ~PxTriangleMesh() {}

		virtual	bool					isKindOf(const char* name) const { return !::strcmp("PxTriangleMesh", name) || PxBase::isKindOf(name); }

		bool Is16BitIndices() const
		{
			return mFlags & (1 << 1);		// PxTriangleMeshFlag::e16_BIT_INDICES
		}

		void importExtraData(PxDeserializationContext& context)
		{
			// PT: vertices are followed by indices, so it will be safe to V4Load vertices from a deserialized binary file
			if (mVertices)
				mVertices = context.readExtraData<Vector3, 16>(mNbVertices);

			if (mTriangles)
			{
				if (Is16BitIndices())
					mTriangles = context.readExtraData<uint16_t, 16>(3 * mNbTriangles);
				else
					mTriangles = context.readExtraData<uint32_t, 16>(3 * mNbTriangles);
			}

			if (mExtraTrigData)
				mExtraTrigData = context.readExtraData<uint8_t, 16>(mNbTriangles);

			if (mMaterialIndices)
				mMaterialIndices = context.readExtraData<uint16_t, 16>(mNbTriangles);

			if (mFaceRemap)
				mFaceRemap = context.readExtraData<uint32_t, 16>(mNbTriangles);

			if (mAdjacencies)
				mAdjacencies = context.readExtraData<uint32_t, 16>(3 * mNbTriangles);

			mGRB_triIndices = nullptr;
			mGRB_triAdjacencies = nullptr;
			mGRB_faceRemap = nullptr;
			mGRB_BV32Tree = nullptr;
		}

		uint32_t					mNbVertices;
		uint32_t					mNbTriangles;
		Vector3* mVertices;
		void* mTriangles;
		TCE3<float>				mAABB;
		uint8_t* mExtraTrigData;
		float							mGeomEpsilon;
		uint8_t					mFlags;
		uint16_t* mMaterialIndices;
		uint32_t* mFaceRemap;
		uint32_t* mAdjacencies;
		void* mMeshFactory;

		void* mGRB_triIndices;
		void* mGRB_triAdjacencies;
		uint32_t* mGRB_faceRemap;
		void* mGRB_BV32Tree;
	};


	#define RTREE_N		4

	struct RTreeNodeQ
	{
		float		minx, miny, minz, maxx, maxy, maxz;
		uint32_t	ptr; // lowest bit is leaf flag
	};

	struct RTreePage
	{
		float minx[RTREE_N];
		float miny[RTREE_N];
		float minz[RTREE_N];
		float maxx[RTREE_N];
		float maxy[RTREE_N];
		float maxz[RTREE_N];
		uint32_t ptrs[RTREE_N];
	};

	class alignas(16) RTree
	{
	public:
		Vector4		mBoundsMin, mBoundsMax, mInvDiagonal, mDiagonalScaler; // 16
		uint32_t		mPageSize;
		uint32_t		mNumRootPages;
		uint32_t		mNumLevels;
		uint32_t		mTotalNodes; // 16
		uint32_t		mTotalPages;
		uint32_t		mFlags; enum { USER_ALLOCATED = 0x1, IS_EDGE_SET = 0x2 };
		RTreePage*		mPages;
	};

	struct LeafTriangles
	{
		uint32_t			Data;
	};

	class PxRTreeTriangleMesh : public PxTriangleMesh
	{
	public:
		virtual ~PxRTreeTriangleMesh() { }

		void importExtraData(PxDeserializationContext& context)
		{
			context.alignExtraData(128);
			mRTree.mPages = context.readExtraData<RTreePage>(mRTree.mTotalPages);

			PxTriangleMesh::importExtraData(context);
		}

		void resolveReferences(PxDeserializationContext& context)
		{

		}

		RTree			mRTree;
	};

	static_assert(sizeof(PxBase) == 16, "sizeof(PxBase) not valid");
	static_assert(sizeof(PxRefCountable) == 16, "sizeof(PxRefCountable) not valid");
	static_assert(sizeof(MeshBVH4) == 96, "sizeof(MeshBVH) not valid");
	static_assert(sizeof(PxTriangleMesh) == 160, "sizeof(PxTriangleMesh) not valid");
	static_assert(sizeof(PxRTreeTriangleMesh) == 256, "sizeof(PxRTreeTriangleMesh) not valid");

	struct PxInternalObjectsData
	{
		float	mRadius;
		float	mExtents[3];
	};

	struct GuHullPolygonData
	{
		Plane3d			mPlane;
		uint16_t	mVRef8;
		uint8_t	mNbVerts;
		uint8_t	mMinIndex;
	};

	struct PxValency
	{
		uint16_t		mCount;
		uint16_t		mOffset;
	};

	struct PxBigConvexRawData
	{
		uint16_t		mSubdiv;
		uint16_t		mNbSamples;

		uint8_t* mSamples;

		const uint8_t* getSamples2()	const
		{
			return mSamples + mNbSamples;
		}

		uint32_t		mNbVerts;
		uint32_t		mNbAdjVerts;
		PxValency* mValencies;
		uint8_t* mAdjacentVerts;
	};

	class PxBigConvexData
	{
	public:
		void importExtraData(PxDeserializationContext& context)
		{
			if (mData.mSamples)
				mData.mSamples = context.readExtraData<uint8_t, 16>(uint32_t(mData.mNbSamples * 2));

			if (mData.mValencies)
			{
				context.alignExtraData();
				uint32_t numVerts = (mData.mNbVerts + 3) & ~3;
				mData.mValencies = context.readExtraData<PxValency>(numVerts);
				mData.mAdjacentVerts = context.readExtraData<uint8_t>(mData.mNbAdjVerts);
			}
		}

		PxBigConvexRawData	mData;

	protected:
		void* mVBuffer;
	};

	struct ConvexHullData
	{
		TCE3<float>		mAABB;
		PxVec3			mCenterOfMass;
		PxU16			mNbEdges;
		PxU8			mNbHullVertices;
		PxU8			mNbPolygons;

		PxVec3* getHullVertices()
		{
			char* tmp = reinterpret_cast<char*>(mPolygons);
			tmp += sizeof(GuHullPolygonData) * mNbPolygons;
			return reinterpret_cast<PxVec3*>(tmp);
		}

		PxU8* getFacesByEdges8()
		{
			char* tmp = reinterpret_cast<char*>(mPolygons);
			tmp += sizeof(GuHullPolygonData) * mNbPolygons;
			tmp += sizeof(PxVec3) * mNbHullVertices;
			return reinterpret_cast<PxU8*>(tmp);
		}

		PxU8* getFacesByVertices8()
		{
			char* tmp = reinterpret_cast<char*>(mPolygons);
			tmp += sizeof(GuHullPolygonData) * mNbPolygons;
			tmp += sizeof(PxVec3) * mNbHullVertices;
			tmp += sizeof(PxU8) * mNbEdges * 2;
			return reinterpret_cast<PxU8*>(tmp);
		}

		uint16_t* getVerticesByEdges16()
		{
			if (mNbEdges & 0x8000)
			{
				char* tmp = reinterpret_cast<char*>(mPolygons);
				tmp += sizeof(GuHullPolygonData) * mNbPolygons;
				tmp += sizeof(PxVec3) * mNbHullVertices;
				tmp += sizeof(PxU8) * (mNbEdges & ~0x8000) * 2;
				tmp += sizeof(PxU8) * mNbHullVertices * 3;
				return reinterpret_cast<uint16_t*>(tmp);
			}
			return nullptr;
		}

		PxU8* getVertexData8()
		{
			char* tmp = reinterpret_cast<char*>(mPolygons);
			tmp += sizeof(GuHullPolygonData) * mNbPolygons;
			tmp += sizeof(PxVec3) * mNbHullVertices;
			tmp += sizeof(PxU8) * mNbEdges * 2;
			tmp += sizeof(PxU8) * mNbHullVertices * 3;
			if (mNbEdges & 0x8000)
				tmp += sizeof(PxU16) * mNbEdges * 2;
			return reinterpret_cast<PxU8*>(tmp);
		}

		GuHullPolygonData* mPolygons;
		PxBigConvexRawData* mBigConvexRawData;
		PxInternalObjectsData	mInternal;
	};

	class ConvexMesh : public PxBase, public PxRefCountable
	{
	public:
		virtual	bool				isKindOf(const char* name) const { return !::strcmp("PxConvexMesh", name) || PxBase::isKindOf(name); }

		uint32_t computeBufferSize(const ConvexHullData& data, uint32_t nb)
		{
			uint32_t bytesNeeded = sizeof(GuHullPolygonData) * data.mNbPolygons;
			uint16_t mnbEdges = (data.mNbEdges & ~0x8000);
			bytesNeeded += sizeof(Vector3) * data.mNbHullVertices;
			bytesNeeded += sizeof(uint8_t) * mnbEdges * 2;
			bytesNeeded += sizeof(uint8_t) * data.mNbHullVertices * 3;
			bytesNeeded += (data.mNbEdges & 0x8000) ? (sizeof(uint16_t) * mnbEdges * 2) : 0;
			bytesNeeded += sizeof(uint8_t) * nb;
			const uint32_t mod = bytesNeeded % sizeof(float);
			if (mod)
				bytesNeeded += sizeof(float) - mod;
			return bytesNeeded;
		}

		void importExtraData(PxDeserializationContext& context)
		{
			const uint32_t bufferSize = computeBufferSize(mHullData, GetNb());
			mHullData.mPolygons = reinterpret_cast<GuHullPolygonData*>(context.readExtraData<uint8_t, 16>(bufferSize));

			assert(mBigConvexData == nullptr);
			if (mBigConvexData)
			{
				mBigConvexData = context.readExtraData<PxBigConvexData, 16>();
				new(mBigConvexData)PxBigConvexData();
				mBigConvexData->importExtraData(context);
				mHullData.mBigConvexRawData = &mBigConvexData->mData;
			}
		}

		uint32_t GetNb() const
		{
			return mNb & ~0x8000000;
		}

		void resolveReferences(PxDeserializationContext& context)
		{
		}

		ConvexHullData		mHullData;
		uint32_t			mNb;
		PxBigConvexData* mBigConvexData;
		float					mMass;
		Matrix3				mInertia;
		void* mMeshFactory;
	};

	static_assert(sizeof(ConvexHullData) == 72, "sizeof(ConvexHullData) not valid");
	static_assert(sizeof(ConvexMesh) == 168, "sizeof(ConvexMesh) not valid");

	struct PxHeightFieldSample
	{
		uint16_t			height;
		uint8_t	materialIndex0;
		uint8_t	materialIndex1;
	};

	struct HeightFieldData
	{
		TCE3<float>					mAABB;
		uint32_t				rows;
		uint32_t				columns;
		float						rowLimit;
		float						colLimit;
		float						nbColumns;
		PxHeightFieldSample* samples;
		float						convexEdgeThreshold;
		uint16_t				flags;
		int							format;
	};

	typedef PxU16 PxMaterialTableIndex;
	typedef PxU32 PxTriangleID;

	class HeightField : public PxBase, public PxRefCountable
	{
	public:
		virtual ~HeightField() {}
		virtual	bool					isKindOf(const char* name) const { return !::strcmp("PxHeightField", name) || PxBase::isKindOf(name); }

		PxMaterialTableIndex		getTriangleMaterialIndex(PxTriangleID triangleIndex)	const
		{
			return getTriangleMaterial(triangleIndex);
		}

		bool	isFirstTriangle(PxU32 triangleIndex) const { return ((triangleIndex & 0x1) == 0); }

		PxU16	getTriangleMaterial(PxU32 triangleIndex) const
		{
			return isFirstTriangle(triangleIndex) ? getMaterialIndex0(triangleIndex >> 1) : getMaterialIndex1(triangleIndex >> 1);
		}

		PxU16	getMaterialIndex0(PxU32 vertexIndex) const { return mData.samples[vertexIndex].materialIndex0; }
		PxU16	getMaterialIndex1(PxU32 vertexIndex) const { return mData.samples[vertexIndex].materialIndex1; }

		void importExtraData(PxDeserializationContext& context)
		{
			mData.samples = context.readExtraData<PxHeightFieldSample, 16>(mData.rows * mData.columns);
		}

		void resolveReferences(PxDeserializationContext& context)
		{
		}

		HeightFieldData			mData;
		uint32_t				mSampleStride;
		uint32_t				mNbSamples;
		float						mMinHeight;
		float						mMaxHeight;
		uint32_t				mModifyCount;

		void* mMeshFactory;
	};

	static_assert(sizeof(HeightFieldData) == 72, "sizeof(PxHeightFieldData) not valid");
	static_assert(sizeof(HeightField) == 136, "sizeof(PxHeightField) not valid");

	class PxMaterial : public PxBase
	{
	public:
		virtual					~PxMaterial() {}
		virtual		bool			isKindOf(const char* name) const { return !::strcmp("PxMaterial", name) || PxBase::isKindOf(name); }

		void* userData;
	};

	struct alignas(16) PxMaterialCore
	{
		float					dynamicFriction;				//4
		float					staticFriction;					//8
		float					restitution;					//12
		uint16_t			flags;							//14
		uint8_t			fricRestCombineMode;			//15
		uint8_t			padding;						//16
		PxMaterial* mNxMaterial;
		uint16_t			mMaterialIndex; //handle assign by the handle manager
		uint16_t			mPadding;
	};

	class NpMaterial : public PxMaterial, public PxRefCountable
	{
	public:
		virtual				~NpMaterial() {}
		PxU16				getHandle()			const { return mMaterial.mMaterialIndex; }

		void				importExtraData(PxDeserializationContext& context) {}

		void				resolveReferences(PxDeserializationContext& context)
		{
			mMaterial.mNxMaterial = this;
		}
		PxMaterialCore			mMaterial;
	};

	static_assert(sizeof(PxMaterial) == 24, "sizeof(PxMaterial) not valid");
	static_assert(sizeof(PxMaterialCore) == 32, "sizeof(PxMaterialCore) not valid");
	static_assert(sizeof(NpMaterial) == 80, "sizeof(NpMaterial) not valid");

	struct PxFilterData
	{
		PxU32 word0;
		PxU32 word1;
		PxU32 word2;
		PxU32 word3;
	};

	class PxTransform
	{
	public:
		Quaternion q;
		Vector3 p;
	};

	enum Enum
	{
		eSPHERE,
		ePLANE,
		eCAPSULE,
		eBOX,
		eCONVEXMESH,
		eTRIANGLEMESH,
		eHEIGHTFIELD,
		eGEOMETRY_COUNT,	//!< internal use only!
		eINVALID = -1		//!< internal use only!
	};

	class PxGeometry
	{
	public:
		int mType;
	};

	class PxBoxGeometry : public PxGeometry
	{
	public:
		Vector3 halfExtents;
	};

	class PxSphereGeometry : public PxGeometry
	{
	public:
		PxReal radius;
	};

	class PxCapsuleGeometry : public PxGeometry
	{
	public:
		PxReal radius;
		PxReal halfHeight;
	};

	class PxPlaneGeometry : public PxGeometry
	{
	public:
	};

	class PxMeshScale
	{
	public:
		Vector3		scale;
		Quaternion		rotation;
	};

	class PxConvexMeshGeometry : public PxGeometry
	{
	public:
		PxMeshScale			scale;
		ConvexMesh* convexMesh;
		PxU8				meshFlags;
		PxU8				paddingFromFlags[3];
	};

	struct PxConvexMeshGeometryLL : public PxConvexMeshGeometry
	{
		const ConvexHullData* hullData;
		bool				 gpuCompatible;
	};

	class PxTriangleMeshGeometry : public PxGeometry
	{
	public:
		PxMeshScale			scale;
		PxU8				meshFlags;
		PxU8				paddingFromFlags[3];
		PxTriangleMesh* triangleMesh;
	};

	class PxHeightFieldGeometry : public PxGeometry
	{
	public:
		HeightField*		heightField;
		PxReal				heightScale;
		PxReal				rowScale;
		PxReal				columnScale;
		PxU8				heightFieldFlags;
		PxU8				paddingFromFlags[3];
	};

	struct MaterialIndicesStruct
	{
		PxU16* indices;
		PxU16	numIndices;
		PxU16	pad;
#if INTPTR_MAX != INT32_MAX
		PxU32	pad64;
#endif
	};

	struct PxTriangleMeshGeometryLL : public PxTriangleMeshGeometry
	{
		const PxTriangleMesh* meshData;
		const PxU16* materialIndices;
		MaterialIndicesStruct				materials;
	};

	struct PxHeightFieldGeometryLL : public PxHeightFieldGeometry
	{
		const HeightFieldData* heightFieldData;
		MaterialIndicesStruct		materials;
	};

	class InvalidGeometry : public PxGeometry
	{
	public:
	};

	class GeometryUnion
	{
	public:
		int getType()					const { return reinterpret_cast<const PxGeometry&>(mGeometry).mType; }

		template<class Geom> Geom& get()
		{
			return reinterpret_cast<Geom&>(mGeometry);
		}

		const PxGeometry& getGeometry()				const { return reinterpret_cast<const PxGeometry&>(mGeometry); }

		union {
			void* alignment;	// PT: Makes sure the class is at least aligned to pointer size. See DE6803. 
			PxU8	box[sizeof(PxBoxGeometry)];
			PxU8	sphere[sizeof(PxSphereGeometry)];
			PxU8	capsule[sizeof(PxCapsuleGeometry)];
			PxU8	plane[sizeof(PxPlaneGeometry)];
			PxU8	convex[sizeof(PxConvexMeshGeometryLL)];
			PxU8	mesh[sizeof(PxTriangleMeshGeometryLL)];
			PxU8	heightfield[sizeof(PxHeightFieldGeometryLL)];
			PxU8	invalid[sizeof(InvalidGeometry)];
		} mGeometry;
	};

	struct PxsShapeCore
	{
		alignas(16) PxTransform			transform;
		PxReal				contactOffset;
		PxU8				mShapeFlags;			// !< API shape flags	// PT: TODO: use PxShapeFlags here. Needs to move flags to separate file.
		PxU8				mOwnsMaterialIdxMemory;	// PT: for de-serialization to avoid deallocating material index list. Moved there from Sc::ShapeCore (since one byte was free).
		PxU16				materialIndex;
		GeometryUnion		geometry;
	};

	class ScShapeCore
	{
	public:
		void importExtraData(PxDeserializationContext& context)
		{
			const int geomType = mCore.geometry.getType();

			if (geomType == eTRIANGLEMESH)
			{
				MaterialIndicesStruct& materials = mCore.geometry.get<PxTriangleMeshGeometryLL>().materials;
				materials.indices = context.readExtraData<PxU16, 16>(materials.numIndices);
			}
			else if (geomType == eHEIGHTFIELD)
			{
				MaterialIndicesStruct& materials = mCore.geometry.get<PxHeightFieldGeometryLL>().materials;
				materials.indices = context.readExtraData<PxU16, 16>(materials.numIndices);
			}
		}

		PxU16 getNbMaterialIndices()
		{
			const int geomType = mCore.geometry.getType();

			if ((geomType != eTRIANGLEMESH) && (geomType != eHEIGHTFIELD))
			{
				return 1;
			}
			else if (geomType == eTRIANGLEMESH)
			{
				PxTriangleMeshGeometryLL& meshGeom = mCore.geometry.get<PxTriangleMeshGeometryLL>();
				return meshGeom.materials.numIndices;
			}
			else
			{
				assert(geomType == eHEIGHTFIELD);
				PxHeightFieldGeometryLL& hfGeom = mCore.geometry.get<PxHeightFieldGeometryLL>();
				return hfGeom.materials.numIndices;
			}
		}

		const PxU16* getMaterialIndices()
		{
			const int geomType = mCore.geometry.getType();

			if ((geomType != eTRIANGLEMESH) && (geomType != eHEIGHTFIELD))
			{
				return &mCore.materialIndex;
			}
			else if (geomType == eTRIANGLEMESH)
			{
				PxTriangleMeshGeometryLL& meshGeom = mCore.geometry.get<PxTriangleMeshGeometryLL>();
				return meshGeom.materials.indices;
			}
			else
			{
				assert(geomType == eHEIGHTFIELD);
				PxHeightFieldGeometryLL& hfGeom = mCore.geometry.get<PxHeightFieldGeometryLL>();
				return hfGeom.materials.indices;
			}
		}

		void resolveMaterialReference(PxU32 materialTableIndex, PxU16 materialIndex)
		{
			if (materialTableIndex == 0)
			{
				mCore.materialIndex = materialIndex;
			}

			PxGeometry& geom = const_cast<PxGeometry&>(mCore.geometry.getGeometry());

			if (geom.mType == eHEIGHTFIELD)
			{
				PxHeightFieldGeometryLL& hfGeom = static_cast<PxHeightFieldGeometryLL&>(geom);
				hfGeom.materials.indices[materialTableIndex] = materialIndex;
			}
			else if (geom.mType == eTRIANGLEMESH)
			{
				PxTriangleMeshGeometryLL& meshGeom = static_cast<PxTriangleMeshGeometryLL&>(geom);
				meshGeom.materials.indices[materialTableIndex] = materialIndex;
			}
		}

		static PxConvexMeshGeometryLL extendForLL(const PxConvexMeshGeometry& hlGeom)
		{
			PxConvexMeshGeometryLL llGeom;
			static_cast<PxConvexMeshGeometry&>(llGeom) = hlGeom;

			ConvexMesh* cm = static_cast<ConvexMesh*>(hlGeom.convexMesh);

			llGeom.hullData = &cm->mHullData;
			llGeom.gpuCompatible = false;

			return llGeom;
		}

		static PxTriangleMeshGeometryLL extendForLL(const PxTriangleMeshGeometry& hlGeom)
		{
			PxTriangleMeshGeometryLL llGeom;
			static_cast<PxTriangleMeshGeometry&>(llGeom) = hlGeom;

			PxTriangleMesh* tm = static_cast<PxTriangleMesh*>(hlGeom.triangleMesh);
			llGeom.meshData = tm;
			llGeom.materialIndices = tm->mMaterialIndices;
			llGeom.materials = static_cast<const PxTriangleMeshGeometryLL&>(hlGeom).materials;

			return llGeom;
		}

		static PxHeightFieldGeometryLL extendForLL(const PxHeightFieldGeometry& hlGeom)
		{
			PxHeightFieldGeometryLL llGeom;
			static_cast<PxHeightFieldGeometry&>(llGeom) = hlGeom;

			HeightField* hf = static_cast<HeightField*>(hlGeom.heightField);

			llGeom.heightFieldData = &hf->mData;

			llGeom.materials = static_cast<const PxHeightFieldGeometryLL&>(hlGeom).materials;

			return llGeom;
		}


		void resolveReferences(PxDeserializationContext& context)
		{
			// Resolve geometry pointers if needed
			PxGeometry& geom = const_cast<PxGeometry&>(mCore.geometry.getGeometry());

			switch (geom.mType)
			{
			case eCONVEXMESH:
			{
				PxConvexMeshGeometryLL& convexGeom = static_cast<PxConvexMeshGeometryLL&>(geom);
				context.translatePxBase(convexGeom.convexMesh);

				// update the hullData pointer
				static_cast<PxConvexMeshGeometryLL&>(geom) = extendForLL(convexGeom);
			}
			break;

			case eHEIGHTFIELD:
			{
				PxHeightFieldGeometryLL& hfGeom = static_cast<PxHeightFieldGeometryLL&>(geom);
				context.translatePxBase(hfGeom.heightField);

				// update hf pointers
				static_cast<PxHeightFieldGeometryLL&>(geom) = extendForLL(hfGeom);
			}
			break;

			case eTRIANGLEMESH:
			{
				PxTriangleMeshGeometryLL& meshGeom = static_cast<PxTriangleMeshGeometryLL&>(geom);
				context.translatePxBase(meshGeom.triangleMesh);

				// update mesh pointers
				static_cast<PxTriangleMeshGeometryLL&>(geom) = extendForLL(meshGeom);
			}
			break;
			case eSPHERE:
			case ePLANE:
			case eCAPSULE:
			case eBOX:
			case eGEOMETRY_COUNT:
			case eINVALID:
				break;

			}
		}

		PxFilterData				mQueryFilterData;		// Query filter data PT: TODO: consider moving this to SceneQueryShapeData
		PxFilterData				mSimulationFilterData;	// Simulation filter data
		alignas(16) PxsShapeCore	mCore;
		PxReal						mRestOffset;			// same as the API property of the same name
		PxReal						mTorsionalRadius;
		PxReal						mMinTorsionalPatchRadius;
	};

	class ScBase
	{
	public:
		void* mScene;
		uint32_t	mControlState;
		uint8_t* mStreamPtr;
	};

	class ScShape : public ScBase
	{
	public:
		ScShapeCore		mShape;
	};

	class PxShape : public PxBase
	{
	public:
		virtual		bool					isKindOf(const char* name) const { return !::strcmp("PxShape", name) || PxBase::isKindOf(name); }
		void* UserData;
	};

	class NpShape : public PxShape, public PxRefCountable
	{
	public:
		virtual					~NpShape() {}

		int getGeomType() const
		{
			return mShape.mShape.mCore.geometry.getType();
		}

		void importExtraData(PxDeserializationContext& context)
		{
			mShape.mShape.importExtraData(context);
			context.readName(mName);
		}

		void resolveReferences(PxDeserializationContext& context)
		{
			{
				PxU32 nbIndices = mShape.mShape.getNbMaterialIndices();
				const PxU16* indices = mShape.mShape.getMaterialIndices();

				for (PxU32 i = 0; i < nbIndices; i++)
				{
					PxBase* base = context.resolveReference(PX_SERIAL_REF_KIND_MATERIAL_IDX, size_t(indices[i]));
					assert(base && base->is<PxMaterial>());

					NpMaterial& material = *static_cast<NpMaterial*>(base);
					mShape.mShape.resolveMaterialReference(i, material.getHandle());
				}
			}

			context.translatePxBase(mActor);

			mShape.mShape.resolveReferences(context);
		}

		void* mActor;
		ScShape					mShape;
		const char* mName;
		volatile int			mExclusiveAndActorCount;
	};

	static_assert(sizeof(GeometryUnion) == 80, "sizeof(GeometryUnion) not valid");
	static_assert(sizeof(PxsShapeCore) == 128, "sizeof(PxsShapeCore) not valid");
	static_assert(sizeof(ScShapeCore) == 176, "sizeof(ScShapeCore) not valid");
	static_assert(sizeof(ScShape) == 208, "sizeof(ScShape) not valid");
	static_assert(sizeof(NpShape) == 272, "sizeof(NpShape) not valid");

	class ScbBase
	{
	public:
		void* mScene;
		PxU32		mControlState;
		PxU8* mStreamPtr;
	};

	class ScbActor : public ScbBase
	{
	public:
	};

	class ScbRigidObject : public ScbActor
	{
	public:
	};

	class ScActorCore
	{
	public:
		void* mSim;
		PxU32				mAggregateIDOwnerClient;
		PxU8				mActorFlags;	// PxActorFlags
		PxU8				mActorType;
		PxU8				mDominanceGroup;
	};

	class ScRigidCore : public ScActorCore
	{
	public:
	};

	struct PxsRigidCore
	{
		alignas(16) PxTransform		body2World;
		PxU16						mFlags;
		PxU16						solverIterationCounts;
	};

	struct PxsBodyCore : public PxsRigidCore
	{
	protected:
		PxTransform				body2Actor;
	public:
		PxReal					ccdAdvanceCoefficient;	//64

		PxVec3					linearVelocity;
		PxReal					maxPenBias;

		PxVec3					angularVelocity;
		PxReal					contactReportThreshold;	//96

		PxReal					maxAngularVelocitySq;
		PxReal					maxLinearVelocitySq;
		PxReal					linearDamping;
		PxReal					angularDamping;			//112

		PxVec3					inverseInertia;
		PxReal					inverseMass;			//128

		PxReal					maxContactImpulse;
		PxReal					sleepThreshold;
		PxReal					freezeThreshold;
		PxReal					wakeCounter;
		PxReal					solverWakeCounter;
		PxU32					numCountedInteractions;
		PxU32					numBodyInteractions;
		PxU8					isFastMoving;
		PxU8					disableGravity;
		PxU8					lockFlags;
		PxU8					kinematicLink;
	};

	class ScStaticCore : public ScRigidCore
	{
	public:
		PxsRigidCore		mCore;
	};

	class RigidStatic : public ScbRigidObject
	{
	public:
		ScStaticCore		mStatic;
	};

	class BodyCore : public ScRigidCore
	{
	public:
		alignas(16) PxsBodyCore mCore;
		void*					mSimStateData;			// SimStateData
	};

	class ScbBody : public ScbRigidObject
	{
	public:
		BodyCore			mBodyCore;
		PxTransform			mBufferedBody2World;
		Vector3			mBufferedLinVelocity;
		Vector3			mBufferedAngVelocity;
		PxReal				mBufferedWakeCounter;
		PxU32				mBufferedIsSleeping;
		PxU32				mBodyBufferFlags;
	};

	struct PtrTable
	{
		void* const* getPtrs()	const { return mCount == 1 ? &mSingle : mList; }

		void	importExtraData(PxDeserializationContext& context)
		{
			if (mCount > 1)
				mList = context.readExtraData<void*, 16>(mCount);
		}
		union
		{
			void* mSingle;
			void** mList;
		};

		PxU16	mCount;
		bool	mOwnsMemory;
		bool	mBufferUsed;
	};

	class NpShapeManager
	{
	public:
		NpShape* const* getShapes()			const { return reinterpret_cast<NpShape* const*>(mShapes.getPtrs()); }

		void importExtraData(PxDeserializationContext& context)
		{
			mShapes.importExtraData(context);
			mSceneQueryData.importExtraData(context);
		}

		PtrTable			mShapes;
		PtrTable			mSceneQueryData;
		PxU32 				mSqCompoundId;
		void* mPruningStructure;  // Shape scene query data are pre-build in pruning structure

	};

	class NpActor
	{
	public:
		void importExtraData(PxDeserializationContext& context)
		{
			if (mConnectorArray)
			{
				// TODO
				assert(false);
				/*
				mConnectorArray = context.readExtraData<NpConnectorArray, 16>();
				new (mConnectorArray) NpConnectorArray(PxEmpty);

				if (mConnectorArray->size() == 0)
					mConnectorArray = NULL;
				else
					Cm::importInlineArray(*mConnectorArray, context);
				*/
			}
			context.readName(mName);
		}

		void resolveReferences(PxDeserializationContext& context)
		{
			// Resolve connector pointers if needed
			if (mConnectorArray)
			{
				// TODO
				assert(false);
				/*
				const PxU32 nbConnectors = mConnectorArray->size();
				for (PxU32 i = 0; i < nbConnectors; i++)
				{
					NpConnector& c = (*mConnectorArray)[i];
					context.translatePxBase(c.mObject);
				}
				*/
			}
		}

		const char* mName;
		void* mConnectorArray;		// NpConnectorArray
	};

	template<class APIClass>
	class NpActorTemplate : public APIClass, public NpActor
	{
	public:
		virtual	void							importExtraData(PxDeserializationContext& context) { NpActor::importExtraData(context); }
		virtual void							resolveReferences(PxDeserializationContext& context) { NpActor::resolveReferences(context); }
	};

	template<class APIClass>
	class NpRigidActorTemplate : public NpActorTemplate<APIClass>
	{
	public:
		int GetNumShapes() const
		{
			return mShapeManager.mShapes.mCount;
		}

		NpShape* const* GetShapes() const
		{
			return mShapeManager.getShapes();
		}

		void importExtraData(PxDeserializationContext& context)
		{
			mShapeManager.importExtraData(context);
			NpActorTemplate<APIClass>::importExtraData(context);
		}

		void resolveReferences(PxDeserializationContext& context)
		{
			const PxU32 nbShapes = mShapeManager.mShapes.mCount;
			NpShape** shapes = const_cast<NpShape**>(mShapeManager.getShapes());
			for (PxU32 j = 0; j < nbShapes; j++)
			{
				context.translatePxBase(shapes[j]);
				shapes[j]->mActor = this;		// shapes[j]->onActorAttach(*this);
			}

			NpActorTemplate<APIClass>::resolveReferences(context);
		}

		NpShapeManager			mShapeManager;
		PxU32					mIndex;    // index for the NpScene rigid actor array
	};

	class NpRigidStatic : public NpRigidActorTemplate<PxRigidStatic>
	{
	public:
		virtual				~NpRigidStatic() {}
		RigidStatic 		mRigidStatic;
	};

	template<class APIClass>
	class NpRigidBodyTemplate : public NpRigidActorTemplate<APIClass>
	{
	public:
		virtual	~NpRigidBodyTemplate() {}
		ScbBody 			mBody;
	};

	class NpRigidDynamic : public NpRigidBodyTemplate<PxRigidDynamic>
	{
	public:
		virtual							~NpRigidDynamic() {}
	};

	static_assert(sizeof(ScbBase) == 24, "sizeof(ScbBase) not valid");
	static_assert(sizeof(ScbActor) == 24, "sizeof(ScbActor) not valid");
	static_assert(sizeof(ScbRigidObject) == 24, "sizeof(ScbRigidObject) not valid");

	static_assert(sizeof(ScActorCore) == 16, "sizeof(ScActorCore) not valid");
	static_assert(sizeof(ScRigidCore) == 16, "sizeof(ScRigidCore) not valid");
	static_assert(sizeof(PxsRigidCore) == 32, "sizeof(PxsRigidCore) not valid");
	static_assert(sizeof(ScStaticCore) == 48, "sizeof(ScStaticCore) not valid");

	static_assert(sizeof(PxRigidStatic) == 24, "sizeof(PxRigidStatic) not valid");
	static_assert(sizeof(RigidStatic) == 80, "sizeof(RigidStatic) not valid");
	static_assert(sizeof(NpRigidStatic) == 176, "sizeof(NpRigidStatic) not valid");

	static_assert(sizeof(ScbBody) == 288, "sizeof(ScbBody) not valid");
	static_assert(sizeof(NpRigidDynamic) == 384, "sizeof(NpRigidDynamic) not valid");

#define SN_BINARY_VERSION_GUID_NUM_CHARS 32
#define PX_BINARY_SERIAL_VERSION "77E92B17A4084033A0FDB51332D5A6BB"

	bool checkCompatibility(const char* binaryVersionGuidCandidate)
	{
		for (int i = 0; i < SN_BINARY_VERSION_GUID_NUM_CHARS; i++)
		{
			if (binaryVersionGuidCandidate[i] != PX_BINARY_SERIAL_VERSION[i])
			{
				return false;
			}
		}
		return true;
	}

	const char* getBinaryVersionGuid()
	{
		return PX_BINARY_SERIAL_VERSION;
	}

	bool readHeader(uint8_t*& address, PxU32& version, BinaryPlatform& platform)
	{
		const uint32_t header = read32(address);
		version = read32(address);
		char binaryVersionGuid[SN_BINARY_VERSION_GUID_NUM_CHARS + 1];
		memcpy(binaryVersionGuid, address, SN_BINARY_VERSION_GUID_NUM_CHARS);
		binaryVersionGuid[SN_BINARY_VERSION_GUID_NUM_CHARS] = 0;
		address += SN_BINARY_VERSION_GUID_NUM_CHARS;

		const uint32_t platformTag = read32(address);
		const uint32_t markedPadding = read32(address);

		if (header != MAKE_FOURCC('S', 'E', 'B', 'D'))
		{
			printf("Buffer contains data with wrong header indicating invalid binary data. header : %u\n", header);
			return false;
		}

		if (!checkCompatibility(binaryVersionGuid))
		{
			printf("Buffer contains binary data version 0x%s and is incompatible with this PhysX sdk (0x%s).\n",
				binaryVersionGuid, getBinaryVersionGuid());
			return false;
		}

		return true;
	}

	template <class T>
	PxBase* DeserializePhysxObj(uint8_t*& address, PxDeserializationContext& context)
	{
		T* obj = new (address) T;
		obj->importExtraData(context);
		obj->resolveReferences(context);
		address += sizeof(T);
		return obj;
	}

	PxBase* Deserialize(uint8_t*& address, PxDeserializationContext& context, int classType)
	{
		PxBase* instance = nullptr;
		if (classType == PxConcreteType::eTRIANGLE_MESH_BVH33)
		{
			instance = DeserializePhysxObj<PxRTreeTriangleMesh>(address, context);
		}
		else if (classType == PxConcreteType::eCONVEX_MESH)
		{
			instance = DeserializePhysxObj<ConvexMesh>(address, context);
		}
		else if (classType == PxConcreteType::e_HEIGHTFIELD)
		{
			instance = DeserializePhysxObj<HeightField>(address, context);
		}
		else if (classType == PxConcreteType::eMATERIAL)
		{
			instance = DeserializePhysxObj<NpMaterial>(address, context);
		}
		else if (classType == PxConcreteType::eRIGID_STATIC)
		{
			instance = DeserializePhysxObj<NpRigidStatic>(address, context);
		}
		else if (classType == PxConcreteType::eRIGID_DYNAMIC)
		{
			instance = DeserializePhysxObj<NpRigidDynamic>(address, context);
		}
		else if (classType == PxConcreteType::eSHAPE)
		{
			instance = DeserializePhysxObj<NpShape>(address, context);
		}
		return instance;
	}
};
