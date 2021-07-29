#pragma once

#include <unordered_map>
#include <vector>

class PhysxFormat_41
{
public:
	enum PxType
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

	struct ManifestEntry
	{
		ManifestEntry(unsigned int _offset, PxType _type)
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

		unsigned int offset;
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
		SerialObjectIndex(unsigned int index, bool external) { setIndex(index, external); }
		SerialObjectIndex(const SerialObjectIndex& objIndex) : mObjIndex(objIndex.mObjIndex) {}
		SerialObjectIndex() : mObjIndex(0xFFFFFFFF) {}

		void setIndex(unsigned int index, bool external)
		{
			assert((index & SERIAL_OBJECT_INDEX_TYPE_BIT) == 0);
			mObjIndex = index | (external ? SERIAL_OBJECT_INDEX_TYPE_BIT : 0);
		}

		unsigned int getIndex(bool& isExternal)
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
		unsigned int mObjIndex;
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
		unsigned int pad;
#endif
	};

	struct InternalReferenceHandle16
	{
		InternalReferenceHandle16() {}

		InternalReferenceHandle16(unsigned short _reference, SerialObjectIndex _objIndex) :
			reference(_reference),
			pad(0xcdcd),
			objIndex(_objIndex)
		{
		}

		unsigned short reference;
		unsigned short pad;
		SerialObjectIndex objIndex;
	};

	template<typename T> struct PxTypeInfo {};

#define PX_DEFINE_TYPEINFO(_name, _fastType) \
	class _name; \
	template <> struct PxTypeInfo<_name>	{	static const char* name() { return #_name;	}	enum { eFastTypeId = _fastType };	};

	PX_DEFINE_TYPEINFO(PxBase, eUNDEFINED)
		PX_DEFINE_TYPEINFO(PxMaterial, eMATERIAL)
		PX_DEFINE_TYPEINFO(PxConvexMesh, eCONVEX_MESH)
		PX_DEFINE_TYPEINFO(PxTriangleMesh, eUNDEFINED)
		PX_DEFINE_TYPEINFO(PxBVH33TriangleMesh, eTRIANGLE_MESH_BVH33)
		PX_DEFINE_TYPEINFO(PxBVH34TriangleMesh, eTRIANGLE_MESH_BVH34)
		PX_DEFINE_TYPEINFO(PxHeightField, e_HEIGHTFIELD)
		PX_DEFINE_TYPEINFO(PxActor, eUNDEFINED)
		PX_DEFINE_TYPEINFO(PxRigidActor, eUNDEFINED)
		PX_DEFINE_TYPEINFO(PxRigidBody, eUNDEFINED)
		PX_DEFINE_TYPEINFO(PxRigidDynamic, eRIGID_DYNAMIC)
		PX_DEFINE_TYPEINFO(PxRigidStatic, eRIGID_STATIC)
		PX_DEFINE_TYPEINFO(PxArticulationLink, eARTICULATION_LINK)
		PX_DEFINE_TYPEINFO(PxArticulationJoint, eARTICULATION_JOINT)
		PX_DEFINE_TYPEINFO(PxArticulationJointReducedCoordinate, eARTICULATION_JOINT_REDUCED_COORDINATE)
		PX_DEFINE_TYPEINFO(PxArticulation, eARTICULATION)
		PX_DEFINE_TYPEINFO(PxArticulationReducedCoordinate, eARTICULATION_REDUCED_COORDINATE)
		PX_DEFINE_TYPEINFO(PxAggregate, eAGGREGATE)
		PX_DEFINE_TYPEINFO(PxConstraint, eCONSTRAINT)
		PX_DEFINE_TYPEINFO(PxShape, eSHAPE)
		PX_DEFINE_TYPEINFO(PxPruningStructure, ePRUNING_STRUCTURE)

		class PxBase
	{
	public:
		virtual				~PxBase() {};
		virtual				bool		isKindOf(const char* superClass) const { return !::strcmp(superClass, "PxBase"); }

		PxType				getConcreteType() const { return (PxType)mConcreteType; }
		template<class T> T* is() { return typeMatch<T>() ? static_cast<T*>(this) : NULL; }
		template<class T>	bool	typeMatch() const
		{
			return PxU32(PxTypeInfo<T>::eFastTypeId) != PxU32(eUNDEFINED) ?
				PxU32(getConcreteType()) == PxU32(PxTypeInfo<T>::eFastTypeId) : isKindOf(PxTypeInfo<T>::name());
		}

		unsigned short		mConcreteType;			// concrete type identifier - see PxConcreteType.
		unsigned short		mBaseFlags;				// internal flags
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
			unsigned char* objectDataAddress,
			const std::unordered_map<size_t, SerialObjectIndex>& internalPtrReferencesMap,
			const std::unordered_map<unsigned short, SerialObjectIndex>& internalHandle16ReferencesMap,
			// const Cm::Collection* externalRefs,
			unsigned char* extraData)
			: mManifestTable(manifestTable)
			, mImportReferences(importReferences)
			, mObjectDataAddress(objectDataAddress)
			, mInternalPtrReferencesMap(internalPtrReferencesMap)
			, mInternalHandle16ReferencesMap(internalHandle16ReferencesMap)
			// , mExternalRefs(externalRefs)
		{
			mExtraDataAddress = extraData;
		}

		~PxDeserializationContext() {}

		void			readName(const char*& name)
		{
			unsigned int len = *reinterpret_cast<unsigned int*>(mExtraDataAddress);
			mExtraDataAddress += sizeof(len);
			name = len ? reinterpret_cast<const char*>(mExtraDataAddress) : NULL;
			mExtraDataAddress += len;
		}

		template<typename T>
		T* readExtraData(unsigned int count = 1)
		{
			T* data = reinterpret_cast<T*>(mExtraDataAddress);
			mExtraDataAddress += sizeof(T) * count;
			return data;
		}

		template<typename T, unsigned int alignment>
		T* readExtraData(unsigned int count = 1)
		{
			alignExtraData(alignment);
			return readExtraData<T>(count);
		}

		void			alignExtraData(unsigned int alignment = 16)
		{
			size_t addr = reinterpret_cast<size_t>(mExtraDataAddress);
			addr = (addr + alignment - 1) & ~size_t(alignment - 1);
			mExtraDataAddress = reinterpret_cast<unsigned char*>(addr);
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
		unsigned char* mExtraDataAddress;

		const ManifestEntry* mManifestTable;
		const ImportReference* mImportReferences;
		unsigned char* mObjectDataAddress;

		//internal references maps for resolving references.
		const std::unordered_map<size_t, SerialObjectIndex>& mInternalPtrReferencesMap;
		const std::unordered_map<unsigned short, SerialObjectIndex>& mInternalHandle16ReferencesMap;

		//external collection for resolving import references.
		// const Cm::Collection* mExternalRefs;
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
				mVertices = context.readExtraData<Vector3d, 16>(mNbVertices);

			if (mTriangles)
			{
				if (Is16BitIndices())
					mTriangles = context.readExtraData<unsigned short, 16>(3 * mNbTriangles);
				else
					mTriangles = context.readExtraData<unsigned int, 16>(3 * mNbTriangles);
			}

			if (mExtraTrigData)
				mExtraTrigData = context.readExtraData<unsigned char, 16>(mNbTriangles);

			if (mMaterialIndices)
				mMaterialIndices = context.readExtraData<unsigned short, 16>(mNbTriangles);

			if (mFaceRemap)
				mFaceRemap = context.readExtraData<unsigned int, 16>(mNbTriangles);

			if (mAdjacencies)
				mAdjacencies = context.readExtraData<unsigned int, 16>(3 * mNbTriangles);

			mGRB_triIndices = nullptr;
			mGRB_triAdjacencies = nullptr;
			mGRB_faceRemap = nullptr;
			mGRB_BV32Tree = nullptr;
		}

		unsigned int					mNbVertices;
		unsigned int					mNbTriangles;
		Vector3d* mVertices;
		void* mTriangles;
		TCE3<float>				mAABB;
		unsigned char* mExtraTrigData;
		float							mGeomEpsilon;
		unsigned char					mFlags;
		unsigned short* mMaterialIndices;
		unsigned int* mFaceRemap;
		unsigned int* mAdjacencies;
		void* mMeshFactory;

		void* mGRB_triIndices;
		void* mGRB_triAdjacencies;
		unsigned int* mGRB_faceRemap;
		void* mGRB_BV32Tree;
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

		RTree				mRTree;
	};

	static_assert(sizeof(PxBase) == 16, "sizeof(PxBase) not valid");
	static_assert(sizeof(PxRefCountable) == 16, "sizeof(PxRefCountable) not valid");
	static_assert(sizeof(RTree) == 96, "sizeof(RTree) not valid");
	static_assert(sizeof(PxTriangleMesh) == 160, "sizeof(PxTriangleMesh) not valid");
	static_assert(sizeof(PxRTreeTriangleMesh) == 256, "sizeof(PxRTreeTriangleMesh) not valid");

	struct PxInternalObjectsData
	{
		float	mRadius;
		float	mExtents[3];
	};

	struct PxHullPolygonData
	{
		Plane3d			mPlane;
		unsigned short	mVRef8;
		unsigned char	mNbVerts;
		unsigned char	mMinIndex;
	};

	struct PxValency
	{
		unsigned short		mCount;
		unsigned short		mOffset;
	};

	struct PxBigConvexRawData
	{
		unsigned short		mSubdiv;
		unsigned short		mNbSamples;

		unsigned char* mSamples;

		const unsigned char* getSamples2()	const
		{
			return mSamples + mNbSamples;
		}

		unsigned int		mNbVerts;
		unsigned int		mNbAdjVerts;
		PxValency* mValencies;
		unsigned char* mAdjacentVerts;
	};

	class PxBigConvexData
	{
	public:
		void importExtraData(PxDeserializationContext& context)
		{
			if (mData.mSamples)
				mData.mSamples = context.readExtraData<unsigned char, 16>(unsigned int(mData.mNbSamples * 2));

			if (mData.mValencies)
			{
				context.alignExtraData();
				unsigned int numVerts = (mData.mNbVerts + 3) & ~3;
				mData.mValencies = context.readExtraData<PxValency>(numVerts);
				mData.mAdjacentVerts = context.readExtraData<unsigned char>(mData.mNbAdjVerts);
			}
		}

		PxBigConvexRawData	mData;

	protected:
		void* mVBuffer;
	};

	struct ConvexHullData
	{
		TCE3<float>		mAABB;
		Vector3d				mCenterOfMass;
		unsigned short			mNbEdges;
		unsigned char			mNbHullVertices;
		unsigned char			mNbPolygons;

		const Vector3d* getVerts()	const
		{
			const char* tmp = reinterpret_cast<const char*>(mPolygons);
			tmp += sizeof(PxHullPolygonData) * mNbPolygons;
			return reinterpret_cast<const Vector3d*>(tmp);
		}

		const unsigned short* getVerticesByEdges16() const
		{
			if (mNbEdges & 0x8000)
			{
				const char* tmp = reinterpret_cast<const char*>(mPolygons);
				tmp += sizeof(PxHullPolygonData) * mNbPolygons;
				tmp += sizeof(Vector3d) * mNbHullVertices;
				tmp += sizeof(unsigned char) * (mNbEdges & ~0x8000) * 2;
				tmp += sizeof(unsigned char) * mNbHullVertices * 3;
				return reinterpret_cast<const unsigned short*>(tmp);
			}
			return nullptr;
		}

		PxHullPolygonData* mPolygons;
		PxBigConvexRawData* mBigConvexRawData;
		PxInternalObjectsData	mInternal;
	};

	class PxConvexMesh : public PxBase, public PxRefCountable
	{
	public:
		virtual	bool				isKindOf(const char* name) const { return !::strcmp("PxConvexMesh", name) || PxBase::isKindOf(name); }

		unsigned int computeBufferSize(const ConvexHullData& data, unsigned int nb)
		{
			unsigned int bytesNeeded = sizeof(PxHullPolygonData) * data.mNbPolygons;
			unsigned short mnbEdges = (data.mNbEdges & ~0x8000);
			bytesNeeded += sizeof(Vector3d) * data.mNbHullVertices;
			bytesNeeded += sizeof(unsigned char) * mnbEdges * 2;
			bytesNeeded += sizeof(unsigned char) * data.mNbHullVertices * 3;
			bytesNeeded += (data.mNbEdges & ~0x8000) ? (sizeof(unsigned short) * mnbEdges * 2) : 0;
			bytesNeeded += sizeof(unsigned char) * nb;
			const unsigned int mod = bytesNeeded % sizeof(float);
			if (mod)
				bytesNeeded += sizeof(float) - mod;
			return bytesNeeded;
		}

		void importExtraData(PxDeserializationContext& context)
		{
			const unsigned int bufferSize = computeBufferSize(mHullData, GetNb());
			mHullData.mPolygons = reinterpret_cast<PxHullPolygonData*>(context.readExtraData<unsigned char, 16>(bufferSize));

			assert(mBigConvexData == nullptr);
			if (mBigConvexData)
			{
				mBigConvexData = context.readExtraData<PxBigConvexData, 16>();
				new(mBigConvexData)PxBigConvexData();
				mBigConvexData->importExtraData(context);
				mHullData.mBigConvexRawData = &mBigConvexData->mData;
			}
		}

		unsigned int GetNb() const
		{
			return mNb & ~0x8000000;
		}

		void resolveReferences(PxDeserializationContext& context)
		{
		}

		ConvexHullData		mHullData;
		unsigned int			mNb;
		PxBigConvexData* mBigConvexData;
		float					mMass;
		Matrix3d				mInertia;
		void* mMeshFactory;
	};

	static_assert(sizeof(ConvexHullData) == 72, "sizeof(ConvexHullData) not valid");
	static_assert(sizeof(PxConvexMesh) == 168, "sizeof(ConvexMesh) not valid");

	struct PxHeightFieldSample
	{
		unsigned short			height;
		unsigned char	materialIndex0;
		unsigned char	materialIndex1;
	};

	struct HeightFieldData
	{
		TCE3<float>					mAABB;
		unsigned int				rows;
		unsigned int				columns;
		float						rowLimit;
		float						colLimit;
		float						nbColumns;
		PxHeightFieldSample* samples;
		float						convexEdgeThreshold;
		unsigned short				flags;
		int							format;
	};

	class PxHeightField : public PxBase, public PxRefCountable
	{
	public:
		virtual ~PxHeightField() {}
		virtual	bool					isKindOf(const char* name) const { return !::strcmp("PxHeightField", name) || PxBase::isKindOf(name); }

		void importExtraData(PxDeserializationContext& context)
		{
			mData.samples = context.readExtraData<PxHeightFieldSample, 16>(mData.rows * mData.columns);
		}

		void resolveReferences(PxDeserializationContext& context)
		{
		}

		HeightFieldData			mData;
		unsigned int				mSampleStride;
		unsigned int				mNbSamples;
		float						mMinHeight;
		float						mMaxHeight;
		unsigned int				mModifyCount;

		void* mMeshFactory;
	};

	static_assert(sizeof(HeightFieldData) == 72, "sizeof(PxHeightFieldData) not valid");
	static_assert(sizeof(PxHeightField) == 136, "sizeof(PxHeightField) not valid");

	class PxMaterial : public PxBase
	{
	public:
		virtual					~PxMaterial() {}
		virtual		bool			isKindOf(const char* name) const { return !::strcmp("PxMaterial", name) || PxBase::isKindOf(name); }

		void* userData;
	};

	__declspec(align(16))
		struct PxMaterialCore
	{
		float					dynamicFriction;				//4
		float					staticFriction;					//8
		float					restitution;					//12
		unsigned short			flags;							//14
		unsigned char			fricRestCombineMode;			//15
		unsigned char			padding;						//16
		PxMaterial* mNxMaterial;
		unsigned short			mMaterialIndex; //handle assign by the handle manager
		unsigned short			mPadding;
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
		PxConvexMesh* convexMesh;
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
		PxHeightField* heightField;
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
		__declspec(align(16)) PxTransform			transform;
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

			PxConvexMesh* cm = static_cast<PxConvexMesh*>(hlGeom.convexMesh);

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

			PxHeightField* hf = static_cast<PxHeightField*>(hlGeom.heightField);

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
		PxsShapeCore				__declspec(align(16)) mCore;
		PxReal						mRestOffset;			// same as the API property of the same name
		PxReal						mTorsionalRadius;
		PxReal						mMinTorsionalPatchRadius;
	};

	class ScBase
	{
	public:
		void* mScene;
		unsigned int	mControlState;
		unsigned char* mStreamPtr;
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
		__declspec(align(16)) PxTransform			body2World;
		PxU16				mFlags;
		PxU16				solverIterationCounts;
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
		__declspec(align(16)) PxsBodyCore mCore;
		void* mSimStateData;			// SimStateData
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

	bool readHeader(unsigned char*& address)
	{
		const unsigned int header = read32(address);
		const unsigned int version = read32(address);
		char binaryVersionGuid[SN_BINARY_VERSION_GUID_NUM_CHARS + 1];
		memcpy(binaryVersionGuid, address, SN_BINARY_VERSION_GUID_NUM_CHARS);
		binaryVersionGuid[SN_BINARY_VERSION_GUID_NUM_CHARS] = 0;
		address += SN_BINARY_VERSION_GUID_NUM_CHARS;

		const unsigned int platformTag = read32(address);
		const unsigned int markedPadding = read32(address);

		if (header != MAKE_FOURCC('S', 'E', 'B', 'D'))
		{
			printf("Buffer contains data with wrong header indicating invalid binary data.");
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

	bool DeserializeFromBuffer(void* Buffer, Collections& collection)
	{
		if (size_t(Buffer) & (128 - 1))
		{
			printf("Buffer must be 128-bytes aligned.\n");
			return false;
		}

		unsigned char* address = reinterpret_cast<unsigned char*>(Buffer);

		if (!readHeader(address))
		{
			return false;
		}

		ManifestEntry* manifestTable;
		unsigned int nbObjectsInCollection;
		unsigned int objectDataEndOffset;

		// read number of objects in collection
		address = alignPtr(address);
		nbObjectsInCollection = read32(address);

		// read manifest (unsigned int offset, PxConcreteType type)
		{
			address = alignPtr(address);
			unsigned int nbManifestEntries = read32(address);
			assert(*reinterpret_cast<unsigned int*>(address) == 0); //first offset is always 0
			manifestTable = (nbManifestEntries > 0) ? reinterpret_cast<ManifestEntry*>(address) : NULL;
			address += nbManifestEntries * sizeof(ManifestEntry);
			objectDataEndOffset = read32(address);
		}

		ImportReference* importReferences;
		unsigned int nbImportReferences;
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
		unsigned int nbExportReferences;
		// read export references
		{
			address = alignPtr(address);
			nbExportReferences = read32(address);
			exportReferences = (nbExportReferences > 0) ? reinterpret_cast<ExportReference*>(address) : NULL;
			address += nbExportReferences * sizeof(ExportReference);
		}

		// read internal references arrays
		unsigned int nbInternalPtrReferences = 0;
		unsigned int nbInternalHandle16References = 0;
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
			for (unsigned int i = 0; i < nbInternalPtrReferences; i++)
			{
				const InternalReferencePtr& ref = internalPtrReferences[i];
				internalPtrReferencesMap.emplace(ref.reference, SerialObjectIndex(ref.objIndex));
			}
		}

		std::unordered_map<unsigned short, SerialObjectIndex> internalHandle16ReferencesMap(nbInternalHandle16References * 2);
		{
			for (unsigned int i = 0; i < nbInternalHandle16References; i++)
			{
				const InternalReferenceHandle16& ref = internalHandle16References[i];
				internalHandle16ReferencesMap.emplace(ref.reference, SerialObjectIndex(ref.objIndex));
			}
		}

		collection.mObjects.reserve(nbObjectsInCollection * 2);
		if (nbExportReferences > 0)
			collection.mIds.reserve(nbExportReferences * 2);

		unsigned char* addressObjectData = alignPtr(address);
		unsigned char* addressExtraData = alignPtr(addressObjectData + objectDataEndOffset);
		std::unordered_map<PxType, int> Statistic;

		unsigned char* addressExtraDataBase = addressExtraData;
		PxDeserializationContext context(manifestTable, importReferences, addressObjectData, internalPtrReferencesMap, internalHandle16ReferencesMap, addressExtraData);

		FILE* fp = fopen("e://temp//offset.txt", "wb");

		// iterate over memory containing PxBase objects, create the instances, resolve the addresses, import the external data, add to collection.
		{
			unsigned int nbObjects = nbObjectsInCollection;

			while (nbObjects--)
			{
				address = alignPtr(address);
				context.alignExtraData();

				unsigned int Offset = (unsigned int)(context.mExtraDataAddress - addressExtraDataBase);
				fprintf(fp, "%d : offset = %d\n", nbObjectsInCollection - nbObjects, Offset);


				// read PxBase header with type and get corresponding serializer.
				PxBase* header = reinterpret_cast<PxBase*>(address);
				const PxType classType = header->getConcreteType();

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
		}

		fclose(fp);

		for (auto it : Statistic)
		{
			if (it.second >= 0)
			{
				printf("Create class instance for type %d success (%d objs).\n", it.first, it.second);
			}
			else
			{
				printf("Create class instance for type %d failed (%d objs).\n", it.first, -it.second);
			}
		}

		// TODO
		// assert(collection->mObjects.size() == nbObjectsInCollection);
		assert(collection.mObjects.size() <= nbObjectsInCollection);

		// update new collection with export references
		{
			assert(addressObjectData != NULL);
			for (unsigned int i = 0; i < nbExportReferences; i++)
			{
				bool isExternal;
				unsigned int manifestIndex = exportReferences[i].objIndex.getIndex(isExternal);
				assert(!isExternal);
				PxBase* obj = reinterpret_cast<PxBase*>(addressObjectData + manifestTable[manifestIndex].offset);
				collection.mIds.emplace(exportReferences[i].id, obj);
				collection.mObjects[obj] = exportReferences[i].id;
			}
		}
		return true;
	}

	template <class T>
	PxBase* DeserializePhysxObj(unsigned char*& address, PxDeserializationContext& context)
	{
		T src;
		memcpy(&src, address, sizeof(T));
		src.importExtraData(context);
		src.resolveReferences(context);

		T* dst = (T*)address;
		*dst = src;
		address += sizeof(T);
		return dst;
	}

	PxBase* Deserialize(unsigned char*& address, PxDeserializationContext& context, int classType)
	{
		PxBase* instance = nullptr;
		if (classType == eTRIANGLE_MESH_BVH33)
		{
			instance = DeserializePhysxObj<PxRTreeTriangleMesh>(address, context);
		}
		else if (classType == eCONVEX_MESH)
		{
			instance = DeserializePhysxObj<PxConvexMesh>(address, context);
		}
		else if (classType == e_HEIGHTFIELD)
		{
			instance = DeserializePhysxObj<PxHeightField>(address, context);
		}
		else if (classType == eMATERIAL)
		{
			instance = DeserializePhysxObj<NpMaterial>(address, context);
		}
		else if (classType == eRIGID_STATIC)
		{
			instance = DeserializePhysxObj<NpRigidStatic>(address, context);
		}
		else if (classType == eRIGID_DYNAMIC)
		{
			instance = DeserializePhysxObj<NpRigidDynamic>(address, context);
		}
		else if (classType == eSHAPE)
		{
			instance = DeserializePhysxObj<NpShape>(address, context);
		}
		return instance;
	}
};
