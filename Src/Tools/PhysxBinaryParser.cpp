
#include "PhysxBinaryParser.h"
#include "Serialization.h"
#include "../Collision/TriangleMesh.h"
#include "../Collision/RTree.h"
#include "../Maths/Box3d.h"

#include <assert.h>
#include <stdio.h>
#include <unordered_map>
#include <vector>

class PhysxBinaryParser_3_4 : public PhysxBinaryParser
{
public:

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

	class DeserializationContext
	{
	public:
		DeserializationContext(const ManifestEntry* manifestTable,
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

		~DeserializationContext() {}

		void			readName(const char*& name)
		{
			unsigned int len = *reinterpret_cast<unsigned int*>(mExtraDataAddress);
			mExtraDataAddress += sizeof(len);
			name = len ? reinterpret_cast<const char*>(mExtraDataAddress) : NULL;
			mExtraDataAddress += len;
		}

		template<typename T>
		T*				readExtraData(unsigned int count = 1)
		{
			T* data = reinterpret_cast<T*>(mExtraDataAddress);
			mExtraDataAddress += sizeof(T) * count;
			return data;
		}

		template<typename T, unsigned int alignment>
		T*				readExtraData(unsigned int count = 1)
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

	private:
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

	class PxBase
	{
	public:
		virtual				~PxBase() {};
		PxType				getConcreteType() const { return (PxType)mConcreteType; }

		unsigned short		mConcreteType;			// concrete type identifier - see PxConcreteType.
		unsigned short		mBaseFlags;				// internal flags
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

		bool Is16BitIndices() const
		{
			return mFlags & (1 << 1);		// PxTriangleMeshFlag::e16_BIT_INDICES
		}

		void importExtraData(DeserializationContext& context)
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
		Vector3d*						mVertices;
		void*							mTriangles;				
		TAABB3_CE<float>				mAABB;
		unsigned char*					mExtraTrigData;
		float							mGeomEpsilon;
		unsigned char					mFlags;
		unsigned short*					mMaterialIndices;
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

		void importExtraData(DeserializationContext& context)
		{
			context.alignExtraData(128);
			mRTree.mPages = context.readExtraData<RTreePage>(mRTree.mTotalPages);

			PxTriangleMesh::importExtraData(context);
		}

		RTree				mRTree;
	};

	static_assert(sizeof(PxRefCountable) == 16, "sizeof(PxRefCountable) not valid");
	static_assert(sizeof(RTree) == 96, "sizeof(RTree) not valid");
	static_assert(sizeof(PxTriangleMesh) == 160, "sizeof(PxTriangleMesh) not valid");
	static_assert(sizeof(PxRTreeTriangleMesh) == 256, "sizeof(PxRTreeTriangleMesh) not valid");

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


	bool ParseCollectionFromBinary(const char* Filename, Collections* collection)
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
		void* p = (void*)(((((unsigned long long) & buffer[0]) + 127) / 128) * 128);
		fread(p, 1, filesize, fp);
		fclose(fp);

		bool Ret = ParseCollectionFromBuffer(p, collection);
		return Ret;
	}


	bool ParseCollectionFromBuffer(void* Buffer, Collections* collection)
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

		assert(collection);
		collection->mObjects.reserve(nbObjectsInCollection * 2);
		if (nbExportReferences > 0)
			collection->mIds.reserve(nbExportReferences * 2);

		unsigned char* addressObjectData = alignPtr(address);
		unsigned char* addressExtraData = alignPtr(addressObjectData + objectDataEndOffset);
		std::unordered_map<PxType, int> Missing;

		DeserializationContext context(manifestTable, importReferences, addressObjectData, internalPtrReferencesMap, internalHandle16ReferencesMap, addressExtraData);

		// iterate over memory containing PxBase objects, create the instances, resolve the addresses, import the external data, add to collection.
		{
			unsigned int nbObjects = nbObjectsInCollection;

			while (nbObjects--)
			{
				address = alignPtr(address);
				context.alignExtraData();

				// read PxBase header with type and get corresponding serializer.
				PxBase* header = reinterpret_cast<PxBase*>(address);
				const PxType classType = header->getConcreteType();

				void* instance = nullptr;
				if (classType == eTRIANGLE_MESH_BVH33)
				{
					instance = DeserializeTriangleMeshBV33(address, context);
				}
				else if (classType == eCONVEX_MESH)
				{
					instance = DeserializeConvexMesh(address, context);
				}
				else if (classType == eHEIGHTFIELD)
				{
					instance = DeserializeHeightField(address, context);
				}

				if (!instance)
				{
					if (Missing.find(classType) == Missing.end())
					{
						printf("Cannot create class instance for concrete type %d.\n", classType);
						Missing.emplace(classType, 0);
					}
					Missing[classType] += 1;
					continue;
				}

				collection->mObjects.emplace(instance, 0);
			}
		}

		// TODO
		// assert(collection->mObjects.size() == nbObjectsInCollection);
		assert(collection->mObjects.size() <= nbObjectsInCollection);

		// update new collection with export references
		{
			assert(addressObjectData != NULL);
			for (unsigned int i = 0; i < nbExportReferences; i++)
			{
				bool isExternal;
				unsigned int manifestIndex = exportReferences[i].objIndex.getIndex(isExternal);
				assert(!isExternal);
				PxBase* obj = reinterpret_cast<PxBase*>(addressObjectData + manifestTable[manifestIndex].offset);
				collection->mIds.emplace(exportReferences[i].id, obj);
				collection->mObjects[obj] = exportReferences[i].id;
			}
		}
		return true;
	}

	void* DeserializeTriangleMeshBV33(unsigned char*& address, DeserializationContext &context)
	{
		assert(sizeof(RTree) == 96);
		assert(sizeof(PxRTreeTriangleMesh) == 256);

		PxRTreeTriangleMesh pxMesh;
		memcpy(&pxMesh, address, sizeof(PxRTreeTriangleMesh));
		address += sizeof(PxRTreeTriangleMesh);
		pxMesh.importExtraData(context);

		TriangleMesh* Mesh = new TriangleMesh;
		Mesh->CreateEmptyRTree();
		memcpy(Mesh->m_Tree, &pxMesh.mRTree, sizeof(RTree));

		Mesh->SetData(pxMesh.mVertices, pxMesh.mTriangles, pxMesh.mNbVertices, pxMesh.mNbTriangles, pxMesh.Is16BitIndices());
		Mesh->BoundingBox = pxMesh.mAABB.GetMinMax();

		return Mesh;
	}

	void* DeserializeConvexMesh(unsigned char*& address, DeserializationContext& context)
	{
		return nullptr;
	}

	void* DeserializeHeightField(unsigned char*& address, DeserializationContext& context)
	{
		return nullptr;
	}
};

// static
bool PhysxBinaryParser::ParseCollectionFromBinary(const char* Filename, Collections* collection)
{
	PhysxBinaryParser_3_4 parser;
	return parser.ParseCollectionFromBinary(Filename, collection);
}