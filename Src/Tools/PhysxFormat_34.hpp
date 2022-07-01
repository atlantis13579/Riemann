#pragma once

#include <cstddef>
#include <unordered_map>
#include <vector>

namespace PhysxFormat_34
{
	struct PxConcreteType
	{
		enum
		{
			eUNDEFINED,

			eHEIGHTFIELD,
			eCONVEX_MESH,
			eTRIANGLE_MESH_BVH33,
			eTRIANGLE_MESH_BVH34,
			eCLOTH_FABRIC,

			eRIGID_DYNAMIC,
			eRIGID_STATIC,
			eSHAPE,
			eMATERIAL,
			eCONSTRAINT,
			eCLOTH,
			ePARTICLE_SYSTEM,
			ePARTICLE_FLUID,
			eAGGREGATE,
			eARTICULATION,
			eARTICULATION_LINK,
			eARTICULATION_JOINT,
			ePRUNING_STRUCTURE,

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

	PX_DEFINE_TYPEINFO(PxBase, PxConcreteType::eUNDEFINED)
	PX_DEFINE_TYPEINFO(PxMaterial, PxConcreteType::eMATERIAL)
	PX_DEFINE_TYPEINFO(ConvexMesh, PxConcreteType::eCONVEX_MESH)
	PX_DEFINE_TYPEINFO(TriangleMesh, PxConcreteType::eUNDEFINED)
	PX_DEFINE_TYPEINFO(PxBVH33TriangleMesh, PxConcreteType::eTRIANGLE_MESH_BVH33)
	PX_DEFINE_TYPEINFO(PxBVH34TriangleMesh, PxConcreteType::eTRIANGLE_MESH_BVH34)
	PX_DEFINE_TYPEINFO(HeightField, PxConcreteType::eHEIGHTFIELD)
	PX_DEFINE_TYPEINFO(PxActor, PxConcreteType::eUNDEFINED)
	PX_DEFINE_TYPEINFO(PxRigidActor, PxConcreteType::eUNDEFINED)
	PX_DEFINE_TYPEINFO(PxRigidBody, PxConcreteType::eUNDEFINED)
	PX_DEFINE_TYPEINFO(PxRigidDynamic, PxConcreteType::eRIGID_DYNAMIC)
	PX_DEFINE_TYPEINFO(PxRigidStatic, PxConcreteType::eRIGID_STATIC)
	PX_DEFINE_TYPEINFO(PxArticulationLink, PxConcreteType::eARTICULATION_LINK)
	PX_DEFINE_TYPEINFO(PxArticulationJoint, PxConcreteType::eARTICULATION_JOINT)
	PX_DEFINE_TYPEINFO(PxArticulation, PxConcreteType::eARTICULATION)
	PX_DEFINE_TYPEINFO(PxAggregate, PxConcreteType::eAGGREGATE)
	PX_DEFINE_TYPEINFO(PxConstraint, PxConcreteType::eCONSTRAINT)
	PX_DEFINE_TYPEINFO(PxShape, PxConcreteType::eSHAPE)
	PX_DEFINE_TYPEINFO(PxClothFabric, PxConcreteType::eCLOTH_FABRIC)
	PX_DEFINE_TYPEINFO(PxCloth, PxConcreteType::eCLOTH)
	PX_DEFINE_TYPEINFO(PxParticleBase, PxConcreteType::eUNDEFINED)
	PX_DEFINE_TYPEINFO(PxParticleFluid, PxConcreteType::ePARTICLE_FLUID)
	PX_DEFINE_TYPEINFO(PxParticleSystem, PxConcreteType::ePARTICLE_SYSTEM)
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

	enum BinaryPlatform
	{
		UNKNOWN = 0,
		WIN64 = 1,
		LINUX64 = 2,
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
				// const ImportReference& entry = mImportReferences[index];
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

		BinaryPlatform getBinaryPlatform() const
		{
			return mBinaryPlatform;
		}

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

	class TriangleMesh : public PxBase, public PxRefCountable
	{
	public:
		virtual ~TriangleMesh() {}

		virtual	bool					isKindOf(const char* name) const { return !::strcmp("PxTriangleMesh", name) || PxBase::isKindOf(name); }

		bool Is16BitIndices() const
		{
			return mFlags & (1 << 1);		// PxTriangleMeshFlag::e16_BIT_INDICES
		}

		void importExtraData(PxDeserializationContext& context)
		{
			// PT: vertices are followed by indices, so it will be safe to V4Load vertices from a deserialized binary file
			if (mVertices)
				mVertices = context.readExtraData<Vector3d, 16>(mNbVertices);

			if (mTriangles)
			{
				if (mFlags & (1 << 1))
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

		PxU32					mNbVertices;
		PxU32					mNbTriangles;
		PxVec3*					mVertices;
		void*					mTriangles;				//!< 16 (<= 0xffff #vertices) or 32 bit trig indices (mNbTriangles * 3)
		TCE3<float>				mAABB;
		PxU8*					mExtraTrigData;			//one per trig
		PxReal					mGeomEpsilon;			//!< see comments in cooking code referencing this variable
		PxU8					mFlags;					//!< Flag whether indices are 16 or 32 bits wide
														//!< Flag whether triangle adajacencies are build
		PxU16* mMaterialIndices;						//!< the size of the array is numTriangles.
		PxU32* mFaceRemap;								//!< new faces to old faces mapping (after cleaning, etc). Usage: old = faceRemap[new]
		PxU32* mAdjacencies;							//!< Adjacency information for each face - 3 adjacent faces
														//!< Set to 0xFFFFffff if no adjacent face
		void* mMeshFactory;

		// GRB data -------------------------
		void* mGRB_triIndices;							//!< GRB: GPU-friendly tri indices [uint4]

		// TODO avoroshilov: cooking - adjacency info - duplicated, remove it and use 'mAdjacencies' and 'mExtraTrigData' see GuTriangleMesh.cpp:325
		void* mGRB_triAdjacencies;						//!< GRB: adjacency data, with BOUNDARY and NONCONVEX flags (flags replace adj indices where applicable)

		PxU32* mGRB_faceRemap;							//!< GRB : gpu to cpu triangle indice remap
		void* mGRB_BV32Tree;							//!< GRB: BV32 tree
		// End of GRB data ------------------
	};

	#define RTREE_N		4

	struct RTreeNodeQ
	{
		float		minx, miny, minz, maxx, maxy, maxz;
		uint32_t	ptr; // lowest bit is leaf flag
	};

	struct alignas(16) RTreePage
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
		Vector4d		mBoundsMin, mBoundsMax, mInvDiagonal, mDiagonalScaler; // 16
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

	class PxRTreeTriangleMesh : public TriangleMesh
	{
	public:
		PxRTreeTriangleMesh() {	}
		virtual ~PxRTreeTriangleMesh() { }

		void importExtraData(PxDeserializationContext& context)
		{
			context.alignExtraData(128);
			mRTree.mPages = context.readExtraData<RTreePage>(mRTree.mTotalPages);

			TriangleMesh::importExtraData(context);
		}

		void resolveReferences(PxDeserializationContext& context)
		{

		}

		static PxRTreeTriangleMesh* Deserialize(uint8_t*& address, PxDeserializationContext& context)
		{
			PxRTreeTriangleMesh* obj = new (address) PxRTreeTriangleMesh;
			
			// Hack for binary generated by Physx on Linux gcc complier
			if (context.getBinaryPlatform() == LINUX64)
			{
				unsigned char buffer[sizeof(PxRTreeTriangleMesh)];
				memcpy(buffer, address, sizeof(PxRTreeTriangleMesh));

				unsigned char* p = (unsigned char*)obj;
				memcpy(p + OFFSETOF(PxRTreeTriangleMesh, mNbVertices), buffer + 28, sizeof(mNbVertices));
				memcpy(p + OFFSETOF(PxRTreeTriangleMesh, mNbTriangles), buffer + 32, sizeof(mNbTriangles));
				memcpy(p + OFFSETOF(PxRTreeTriangleMesh, mVertices), buffer + 36, sizeof(mVertices));
				memcpy(p + OFFSETOF(PxRTreeTriangleMesh, mTriangles), buffer + 44, sizeof(mTriangles));
				// memcpy(p + offsetof(PxRTreeTriangleMesh, mAABB), buffer + 56, 72);
			}
			obj->importExtraData(context);
			obj->resolveReferences(context);
			address += sizeof(PxRTreeTriangleMesh);
			return obj;
		}

		RTree			mRTree;
	};

	struct InternalObjectsData
	{
		float	mRadius;
		float	mExtents[3];
	};

	struct HullPolygonData
	{
		Plane3d			mPlane;
		PxU16			mVRef8;			//!< Offset of vertex references in hull vertex data (CS: can we assume indices are tightly packed and offsets are ascending?? DrawObjects makes and uses this assumption)
		PxU8			mNbVerts;		//!< Number of vertices/edges in the polygon
		PxU8			mMinIndex;		//!< Index of the polygon vertex that has minimal projection along this plane's normal.
	};

	struct Valency
	{
		PxU16		mCount;
		PxU16		mOffset;
	};

	struct BigConvexRawData
	{
		PxU16				mSubdiv;		// "Gaussmap" subdivision
		PxU16				mNbSamples;		// Total #samples in gaussmap PT: this is not even needed at runtime!

		PxU8* mSamples;
		const PxU8* getSamples2()	const
		{
			return mSamples + mNbSamples;
		}

		// Valencies data
		PxU32		mNbVerts;		//!< Number of vertices
		PxU32		mNbAdjVerts;	//!< Total number of adjacent vertices  ### PT: this is useless at runtime and should not be stored here
		Valency*	mValencies;		//!< A list of mNbVerts valencies (= number of neighbors)
		PxU8*		mAdjacentVerts;	//!< List of adjacent vertices
	};

	static_assert(sizeof(PxBase) == 16, "sizeof(PxBase) not valid");
    static_assert(sizeof(PxRefCountable) == 16, "sizeof(PxRefCountable) not valid");
#if defined(__linux__) || defined(__clang__)
	static_assert(offsetof(PxRTreeTriangleMesh, mNbVertices) == 28, "offset of mNbVertices not right");
#else
	static_assert(offsetof(PxRTreeTriangleMesh, mNbVertices) == 32, "offset of mNbVertices not right");
#endif
    static_assert(sizeof(TriangleMesh) == 160, "sizeof(PxTriangleMesh) not valid");
    static_assert(sizeof(MeshBVH4) == 96, "sizeof(MeshBVH) not valid");
	static_assert(sizeof(PxRTreeTriangleMesh) == 256, "sizeof(PxRTreeTriangleMesh) not valid");
	static_assert(sizeof(InternalObjectsData) == 16, "sizeof(InternalObjectsData) not valid");

	class PxBigConvexData
	{
	public:
		void importExtraData(PxDeserializationContext& context)
		{
			if (mData.mSamples)
				mData.mSamples = context.readExtraData<PxU8, 16>(uint32_t(mData.mNbSamples * 2));

			if (mData.mValencies)
			{
				context.alignExtraData();
				PxU32 numVerts = (mData.mNbVerts + 3) & ~3;
				mData.mValencies = context.readExtraData<Valency>(numVerts);
				mData.mAdjacentVerts = context.readExtraData<PxU8>(mData.mNbAdjVerts);
			}
		}

		BigConvexRawData	mData;

	protected:
		void* mVBuffer;
	};

	struct ConvexHullData
	{
		TCE3<float>		mAABB;
		Vector3d		mCenterOfMass;
		uint16_t		mNbEdges;
		PxU8			mNbHullVertices;
		PxU8			mNbPolygons;

		const Vector3d* getVerts()	const
		{
			const char* tmp = reinterpret_cast<const char*>(mPolygons);
			tmp += sizeof(HullPolygonData) * mNbPolygons;
			return reinterpret_cast<const Vector3d*>(tmp);
		}

		const uint16_t* getVerticesByEdges16() const
		{
			if (mNbEdges & 0x8000)
			{
				const char* tmp = reinterpret_cast<const char*>(mPolygons);
				tmp += sizeof(HullPolygonData) * mNbPolygons;
				tmp += sizeof(Vector3d) * mNbHullVertices;
				tmp += sizeof(uint8_t) * (mNbEdges & ~0x8000) * 2;
				tmp += sizeof(uint8_t) * mNbHullVertices * 3;
				return reinterpret_cast<const uint16_t*>(tmp);
			}
			return nullptr;
		}

		HullPolygonData* mPolygons;
		BigConvexRawData* mBigConvexRawData;
		InternalObjectsData	mInternal;
	};

	class ConvexMesh : public PxBase, public PxRefCountable
	{
	public:
		virtual	bool				isKindOf(const char* name) const { return !::strcmp("PxConvexMesh", name) || PxBase::isKindOf(name); }

		uint32_t computeBufferSize(const ConvexHullData& data, uint32_t nb)
		{
			uint32_t bytesNeeded = sizeof(HullPolygonData) * data.mNbPolygons;
			uint16_t mnbEdges = (data.mNbEdges & ~0x8000);
			bytesNeeded += sizeof(Vector3d) * data.mNbHullVertices;
			bytesNeeded += sizeof(uint8_t) * mnbEdges * 2;
			bytesNeeded += sizeof(uint8_t) * data.mNbHullVertices * 3;
			bytesNeeded += (data.mNbEdges & ~0x8000) ? (sizeof(uint16_t) * mnbEdges * 2) : 0;
			bytesNeeded += sizeof(uint8_t) * nb;
			const uint32_t mod = bytesNeeded % sizeof(float);
			if (mod)
				bytesNeeded += sizeof(float) - mod;
			return bytesNeeded;
		}

		void importExtraData(PxDeserializationContext& context)
		{
			const uint32_t bufferSize = computeBufferSize(mHullData, GetNb());
			mHullData.mPolygons = reinterpret_cast<HullPolygonData*>(context.readExtraData<uint8_t, 16>(bufferSize));

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
		Matrix3d				mInertia;
		void* mMeshFactory;
	};

	static_assert(sizeof(ConvexHullData) == 72, "sizeof(ConvexHullData) not valid");
	static_assert(sizeof(ConvexMesh) == 168, "sizeof(ConvexMesh) not valid");

	struct PxHeightFieldSample
	{
		PxI16			height;
		PxU8			materialIndex0;
		PxU8			materialIndex1;
	};

	struct HeightFieldData
	{
		TCE3<float>					mAABB;
		PxU32						rows;					// PT: WARNING: don't change this member's name (used in ConvX)
		PxU32						columns;				// PT: WARNING: don't change this member's name (used in ConvX)
		PxReal						rowLimit;				// PT: to avoid runtime int-to-float conversions on Xbox
		PxReal						colLimit;				// PT: to avoid runtime int-to-float conversions on Xbox
		PxReal						nbColumns;				// PT: to avoid runtime int-to-float conversions on Xbox
		PxHeightFieldSample*		samples;				// PT: WARNING: don't change this member's name (used in ConvX)
		PxReal						thickness;
		PxReal						convexEdgeThreshold;
		uint16_t					flags;
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
		PxU32					mSampleStride;
		PxU32					mNbSamples;	// PT: added for platform conversion. Try to remove later.
		PxReal					mMinHeight;
		PxReal					mMaxHeight;
		PxU32					mModifyCount;

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

	struct alignas(16) PxsMaterialData
	{
		PxReal					dynamicFriction;				//4
		PxReal					staticFriction;					//8
		PxReal					restitution;					//12
		PxU16					flags;							//14
		PxU8					fricRestCombineMode;			//15
		PxU8					padding;						//16
	};

	class PxsMaterialCore : public PxsMaterialData
	{
	public:
		PxMaterial*				mNxMaterial;
		PxU16					mMaterialIndex; //handle assign by the handle manager
		PxU16					mPadding;
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
		PxsMaterialCore			mMaterial;
	};

	static_assert(sizeof(PxMaterial) == 24, "sizeof(PxMaterial) not valid");
	static_assert(sizeof(PxsMaterialData) == 16, "sizeof(PxsMaterialData) not valid");
	static_assert(sizeof(PxsMaterialCore) == 32, "sizeof(PxMaterialCore) not valid");
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
		Vector3d p;
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
		Vector3d halfExtents;
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
		Vector3d		scale;
		Quaternion		rotation;
	};

	class PxConvexMeshGeometry : public PxGeometry
	{
	public:
		PxMeshScale			scale;
		ConvexMesh*			convexMesh;
		PxReal				maxMargin;			//!< Max shrunk amount permitted by PCM contact gen
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
		TriangleMesh* triangleMesh;
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
		const TriangleMesh* meshData;
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

	class ShapeCore
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

			TriangleMesh* tm = static_cast<TriangleMesh*>(hlGeom.triangleMesh);
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
		ShapeCore		mShape;
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
	static_assert(sizeof(ShapeCore) == 176, "sizeof(ScShapeCore) not valid");
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
		PxU8				mClientBehaviorFlags;
		PxU8				mDominanceGroup;
	};

	class ScRigidCore : public ScActorCore
	{
	public:
	};

	struct PxsRigidCore
	{
		alignas(16) PxTransform		body2World;
		PxU8						mFlags;
		PxU8						mIdtBody2Actor;			// PT: true if PxsBodyCore::body2Actor is identity
		PxU16						solverIterationCounts;	//vel iters are in low word and pos iters in high word.
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
		PxReal					wakeCounter;				//144 this is authoritative wakeCounter

		PxReal					solverWakeCounter;			//this is calculated by the solver when it performs sleepCheck. It is committed to wakeCounter in ScAfterIntegrationTask if the body is still awake.
		PxU32					numCountedInteractions;
		PxU32					numBodyInteractions;		//Used by adaptive force to keep track of the total number of body interactions
		PxU16					isFastMoving;				//This could be a single bit but it's a u32 at the moment for simplicity's sake
		PxU16					lockFlags;					//160 This could be a u8 but it is a u32 for simplicity's sake. All fits into 16 byte alignment
	};

	static_assert(sizeof(PxsBodyCore) == 160, "sizeof(PxsBodyCore) not valid");

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
		Vector3d			mBufferedLinVelocity;
		Vector3d			mBufferedAngVelocity;
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
		void*				mPruningStructure;  // Shape scene query data are pre-build in pruning structure

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

#define SN_NUM_BINARY_COMPATIBLE_VERSIONS 1
#define PX_PHYSICS_VERSION_MAJOR 3
#define PX_PHYSICS_VERSION_MINOR 4
#define PX_PHYSICS_VERSION_BUGFIX 3
#define PX_PHYSICS_VERSION ((PX_PHYSICS_VERSION_MAJOR<<24) + (PX_PHYSICS_VERSION_MINOR<<16) + (PX_PHYSICS_VERSION_BUGFIX<<8) + 0)
#define PX_BINARY_SERIAL_VERSION 0
#define PX_MAKE_FOURCC(a, b, c, d) ( (a) | ((b)<<8) | ((c)<<16) | ((d)<<24) )

	#define SN_NUM_BINARY_PLATFORMS 15
	const PxU32 sBinaryPlatformTags[SN_NUM_BINARY_PLATFORMS] =
	{
		PX_MAKE_FOURCC('W','_','3','2'),
		PX_MAKE_FOURCC('W','_','6','4'),
		PX_MAKE_FOURCC('L','_','3','2'),
		PX_MAKE_FOURCC('L','_','6','4'),
		PX_MAKE_FOURCC('M','_','3','2'),
		PX_MAKE_FOURCC('M','_','6','4'),
		PX_MAKE_FOURCC('M','O','C','A'),
		PX_MAKE_FOURCC('A','N','D','R'),
		PX_MAKE_FOURCC('A','I','O','S'),
		PX_MAKE_FOURCC('A','A','6','4'),
		PX_MAKE_FOURCC('X','O','N','E'),
		PX_MAKE_FOURCC('N','X','3','2'),
		PX_MAKE_FOURCC('N','X','6','4'),
		PX_MAKE_FOURCC('A','D','6','4'),
		PX_MAKE_FOURCC('A','D','8','6'),
	};

	//
	// Important: if you adjust the following structure, please adjust the comment for PX_BINARY_SERIAL_VERSION as well
	//
	const std::pair<PxU32, PxU32> sBinaryCompatibleVersions[SN_NUM_BINARY_COMPATIBLE_VERSIONS] =
	{
		std::pair<PxU32, PxU32>(PX_PHYSICS_VERSION, PX_BINARY_SERIAL_VERSION),
	};

	bool checkCompatibility(const PxU32 version, const PxU32 binaryVersion)
	{
		for (PxU32 i = 0; i < SN_NUM_BINARY_COMPATIBLE_VERSIONS; i++)
		{
			if (version == sBinaryCompatibleVersions[i].first && binaryVersion == sBinaryCompatibleVersions[i].second)
				return true;
		}
		return false;
	}

	bool readHeader(uint8_t*& address, PxU32& version, BinaryPlatform &platform)
	{
		const PxU32 header = read32(address);
		version = read32(address);
		const PxU32 binaryVersion = read32(address);	
		const PxU32 buildNumber = read32(address);
		const PxU32 platformTag = read32(address);
		const PxU32 markedPadding = read32(address);
        
        UNUSED(binaryVersion);
        UNUSED(buildNumber);
        UNUSED(platformTag);
        UNUSED(markedPadding);

		if (header != MAKE_FOURCC('S', 'E', 'B', 'D'))
		{
			printf("Buffer contains data with wrong header indicating invalid binary data. header : %u\n", header);
			return false;
		}

		if (!checkCompatibility(version, binaryVersion))
		{
			printf("Buffer contains binary data version 0x%d and is incompatible with this PhysX sdk (0x%d).\n",
				version, binaryVersion);
			return false;
		}

		if (platformTag == sBinaryPlatformTags[1])
		{
			platform = BinaryPlatform::WIN64;
		}
		else if (platformTag == sBinaryPlatformTags[3])
		{
			platform = BinaryPlatform::LINUX64;
		}
		else
		{
			platform = BinaryPlatform::UNKNOWN;
		}

		return true;
	}

	template <class T>
	PxBase* DeserializePhysxObj(uint8_t*& address, PxDeserializationContext& context)
	{
		T* obj = new (address) T;
		address += sizeof(T);
		obj->importExtraData(context);
		obj->resolveReferences(context);
		return obj;
	}

	PxBase* Deserialize(uint8_t*& address, PxDeserializationContext& context, int classType)
	{
		PxBase* instance = nullptr;
		if (classType == PxConcreteType::eHEIGHTFIELD)
		{
			instance = DeserializePhysxObj<HeightField>(address, context);
		}
		else if (classType == PxConcreteType::eCONVEX_MESH)
		{
			instance = DeserializePhysxObj<ConvexMesh>(address, context);
		}
		else if (classType == PxConcreteType::eTRIANGLE_MESH_BVH33)
		{
			instance = PxRTreeTriangleMesh::Deserialize(address, context);
		}
		else if (classType == PxConcreteType::eTRIANGLE_MESH_BVH34)
		{
			assert(false);
		}
		else if (classType == PxConcreteType::eRIGID_DYNAMIC)
		{
			instance = DeserializePhysxObj<NpRigidDynamic>(address, context);
		}
		else if (classType == PxConcreteType::eRIGID_STATIC)
		{
			instance = DeserializePhysxObj<NpRigidStatic>(address, context);
		}
		else if (classType == PxConcreteType::eSHAPE)
		{
			instance = DeserializePhysxObj<NpShape>(address, context);
		}
		else if (classType == PxConcreteType::eMATERIAL)
		{
			instance = DeserializePhysxObj<NpMaterial>(address, context);
		}
		return instance;
	}

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

};
