
#include "PhysxBinaryParser.h"
#include "Serialization.h"

#include <assert.h>
#include <stdio.h>
#include <unordered_map>
#include <vector>

class PhysxBinaryParser_3_4 : public PhysxBinaryParser
{
public:

	typedef unsigned long long PxSerialObjectId;

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

	class PxBase
	{
	public:
		virtual				~PxBase() {};
		PxType				getConcreteType() const { return (PxType)mConcreteType; }

		unsigned short		mConcreteType;			// concrete type identifier - see PxConcreteType.
		unsigned short		mBaseFlags;				// internal flags
	};

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


	bool ParseCollectionFromBinary(const char* Filename, PhysxObjectDeserializer* pDeserializer)
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

		bool Ret = ParseCollectionFromBuffer(p, pDeserializer);
		return Ret;
	}


	bool ParseCollectionFromBuffer(void* Buffer, PhysxObjectDeserializer* pDeserializer)
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

		/*
		SerializationRegistry& sn = static_cast<SerializationRegistry&>(sr);
		Cm::Collection* collection = static_cast<Cm::Collection*>(PxCreateCollection());
		PX_ASSERT(collection);
		collection->mObjects.reserve(nbObjectsInCollection * 2);
		if (nbExportReferences > 0)
			collection->mIds.reserve(nbExportReferences * 2);
		*/

		unsigned char* addressObjectData = alignPtr(address);
		unsigned char* addressExtraData = alignPtr(addressObjectData + objectDataEndOffset);

		unsigned int nCount = 0;
		// iterate over memory containing PxBase objects, create the instances, resolve the addresses, import the external data, add to collection.
		{
			unsigned int nbObjects = nbObjectsInCollection;

			while (nbObjects--)
			{
				address = alignPtr(address);
				// context.alignExtraData();

				// read PxBase header with type and get corresponding serializer.


				PxBase* header = reinterpret_cast<PxBase*>(address);
				const PxType classType = header->getConcreteType();

				assert(pDeserializer);
				void* instance = pDeserializer->CreateObject(address, classType);
				if (!instance)
				{
					printf("Cannot create class instance for concrete type %d.", classType);
					continue;
				}

				pDeserializer->AddToCollection(instance);
				++nCount;
			}
		}

		// TODO
		// assert(nbObjectsInCollection == nCount);

		// TODO
		assert(nCount <= nbObjectsInCollection);
		// assert(nbExportReferences == 0);

		return true;
	}
};

// static
bool PhysxBinaryParser::ParseCollectionFromBinary(const char* Filename, PhysxObjectDeserializer* pDeserializer)
{
	PhysxBinaryParser_3_4 parser;
	return parser.ParseCollectionFromBinary(Filename, pDeserializer);
}