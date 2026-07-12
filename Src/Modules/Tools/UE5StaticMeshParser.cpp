#include "UE5StaticMeshParser.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_map>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#endif

namespace Riemann
{
namespace
{
	constexpr uint32_t UEPackageTag = 0x9e2a83c1u;
	constexpr uint64_t PackageTrailerHeaderTag = 0xd1c43b2e80a5f697ull;
	constexpr uint64_t PackageTrailerFooterTag = 0x29bfca045138de76ull;
	constexpr uint32_t CompressedBufferTag = 0xb7756362u;

	class Reader
	{
	public:
		Reader(const std::vector<uint8_t>& Data, size_t Begin = 0, size_t End = std::numeric_limits<size_t>::max())
			: mData(Data), mPos(Begin), mEnd(std::min(End, Data.size()))
		{
		}

		bool Seek(size_t Position)
		{
			if (Position > mEnd)
			{
				return Fail("seek outside the input");
			}
			mPos = Position;
			return true;
		}

		bool Skip(size_t Size)
		{
			if (Size > Remaining())
			{
				return Fail("read outside the input");
			}
			mPos += Size;
			return true;
		}

		template <typename T>
		bool Read(T& Value)
		{
			static_assert(std::is_trivially_copyable<T>::value, "Reader only supports trivial values");
			if (sizeof(T) > Remaining())
			{
				return Fail("read outside the input");
			}
			memcpy(&Value, mData.data() + mPos, sizeof(T));
			mPos += sizeof(T);
			return true;
		}

		bool ReadBytes(void* Destination, size_t Size)
		{
			if (Size > Remaining())
			{
				return Fail("read outside the input");
			}
			if (Size != 0)
			{
				memcpy(Destination, mData.data() + mPos, Size);
			}
			mPos += Size;
			return true;
		}

		bool ReadString(std::string& Value)
		{
			int32_t Length = 0;
			if (!Read(Length))
			{
				return false;
			}
			Value.clear();
			if (Length == 0)
			{
				return true;
			}
			if (Length > 0)
			{
				if (static_cast<size_t>(Length) > Remaining())
				{
					return Fail("invalid ANSI string length");
				}
				Value.assign(reinterpret_cast<const char*>(mData.data() + mPos), static_cast<size_t>(Length));
				mPos += static_cast<size_t>(Length);
				if (!Value.empty() && Value.back() == '\0')
				{
					Value.pop_back();
				}
				return true;
			}

			const int64_t WideLength = -static_cast<int64_t>(Length);
			if (WideLength > static_cast<int64_t>(Remaining() / 2))
			{
				return Fail("invalid UTF-16 string length");
			}
			Value.reserve(static_cast<size_t>(WideLength));
			for (int64_t Index = 0; Index < WideLength; ++Index)
			{
				uint16_t Character = 0;
				Read(Character);
				if (Character == 0)
				{
					continue;
				}
				if (Character < 0x80)
				{
					Value.push_back(static_cast<char>(Character));
				}
				else if (Character < 0x800)
				{
					Value.push_back(static_cast<char>(0xc0 | (Character >> 6)));
					Value.push_back(static_cast<char>(0x80 | (Character & 0x3f)));
				}
				else
				{
					Value.push_back(static_cast<char>(0xe0 | (Character >> 12)));
					Value.push_back(static_cast<char>(0x80 | ((Character >> 6) & 0x3f)));
					Value.push_back(static_cast<char>(0x80 | (Character & 0x3f)));
				}
			}
			return true;
		}

		bool Fail(const std::string& Message)
		{
			if (mError.empty())
			{
				std::ostringstream Stream;
				Stream << Message << " at byte " << mPos;
				mError = Stream.str();
			}
			return false;
		}

		size_t Position() const { return mPos; }
		size_t Remaining() const { return mPos <= mEnd ? mEnd - mPos : 0; }
		const uint8_t* Current() const { return mData.data() + mPos; }
		const std::string& Error() const { return mError; }

	private:
		const std::vector<uint8_t>& mData;
		size_t mPos;
		size_t mEnd;
		std::string mError;
	};

	struct PackageExport
	{
		int32_t ClassIndex = 0;
		std::string ObjectName;
		int64_t SerialSize = 0;
		int64_t SerialOffset = 0;
	};

	struct PackageImport
	{
		std::string ObjectName;
	};

	struct Package
	{
		std::vector<uint8_t> Data;
		std::vector<std::string> Names;
		std::vector<PackageImport> Imports;
		std::vector<PackageExport> Exports;
		int32_t UE4Version = 0;
		int32_t UE5Version = 0;
	};

	bool ReadFile(const char* Filename, std::vector<uint8_t>& Data, std::string& Error)
	{
		std::ifstream File(Filename, std::ios::binary | std::ios::ate);
		if (!File)
		{
			Error = "cannot open the UE package";
			return false;
		}
		const std::streamoff Size = File.tellg();
		if (Size <= 0 || static_cast<uint64_t>(Size) > std::numeric_limits<size_t>::max())
		{
			Error = "invalid UE package size";
			return false;
		}
		Data.resize(static_cast<size_t>(Size));
		File.seekg(0, std::ios::beg);
		if (!File.read(reinterpret_cast<char*>(Data.data()), Size))
		{
			Error = "failed to read the UE package";
			return false;
		}
		return true;
	}

	bool ReadPackageName(Reader& Input, const std::vector<std::string>& Names, std::string& Name)
	{
		int32_t Index = 0;
		int32_t Number = 0;
		if (!Input.Read(Index) || !Input.Read(Number) || Index < 0 || static_cast<size_t>(Index) >= Names.size())
		{
			return Input.Fail("invalid package name index");
		}
		Name = Names[static_cast<size_t>(Index)];
		if (Number > 0)
		{
			Name += "_" + std::to_string(Number - 1);
		}
		return true;
	}

	bool ParsePackage(const char* Filename, Package& PackageData, std::string& Error)
	{
		if (!ReadFile(Filename, PackageData.Data, Error))
		{
			return false;
		}
		Reader Input(PackageData.Data);
		uint32_t Tag = 0;
		int32_t LegacyVersion = 0;
		int32_t LicenseeVersion = 0;
		if (!Input.Read(Tag) || Tag != UEPackageTag || !Input.Read(LegacyVersion))
		{
			Error = "not a little-endian UE package";
			return false;
		}
		if (LegacyVersion != -4 && !Input.Skip(4))
		{
			Error = Input.Error();
			return false;
		}
		if (!Input.Read(PackageData.UE4Version) || (LegacyVersion <= -8 && !Input.Read(PackageData.UE5Version)) ||
			!Input.Read(LicenseeVersion))
		{
			Error = Input.Error();
			return false;
		}
		int32_t CustomVersionCount = 0;
		if (!Input.Read(CustomVersionCount) || CustomVersionCount < 0 || CustomVersionCount > 4096 ||
			!Input.Skip(static_cast<size_t>(CustomVersionCount) * 20))
		{
			Error = "invalid package custom version table";
			return false;
		}

		int32_t TotalHeaderSize = 0;
		std::string FolderName;
		uint32_t PackageFlags = 0;
		int32_t NameCount = 0;
		int32_t NameOffset = 0;
		if (!Input.Read(TotalHeaderSize) || !Input.ReadString(FolderName) || !Input.Read(PackageFlags) ||
			!Input.Read(NameCount) || !Input.Read(NameOffset))
		{
			Error = Input.Error();
			return false;
		}
		if (PackageData.UE5Version >= 1003 && !Input.Skip(8))
		{
			Error = Input.Error();
			return false;
		}
		if (PackageData.UE4Version >= 516)
		{
			std::string LocalizationId;
			if (!Input.ReadString(LocalizationId) || !Input.Skip(8))
			{
				Error = Input.Error();
				return false;
			}
		}

		int32_t ExportCount = 0;
		int32_t ExportOffset = 0;
		int32_t ImportCount = 0;
		int32_t ImportOffset = 0;
		int32_t DependsOffset = 0;
		if (!Input.Read(ExportCount) || !Input.Read(ExportOffset) || !Input.Read(ImportCount) ||
			!Input.Read(ImportOffset) || !Input.Read(DependsOffset))
		{
			Error = Input.Error();
			return false;
		}
		if (NameCount < 0 || ExportCount < 0 || ImportCount < 0 || NameOffset < 0 || ExportOffset < 0 ||
			ImportOffset < 0 || DependsOffset < 0 || static_cast<size_t>(DependsOffset) > PackageData.Data.size())
		{
			Error = "invalid UE package table offsets";
			return false;
		}

		Input.Seek(static_cast<size_t>(NameOffset));
		PackageData.Names.reserve(static_cast<size_t>(NameCount));
		for (int32_t Index = 0; Index < NameCount; ++Index)
		{
			std::string Name;
			if (!Input.ReadString(Name) || !Input.Skip(4))
			{
				Error = Input.Error();
				return false;
			}
			PackageData.Names.push_back(std::move(Name));
		}

		const size_t ImportEntrySize = ImportCount > 0 ?
			static_cast<size_t>(ExportOffset - ImportOffset) / static_cast<size_t>(ImportCount) : 0;
		if (ImportCount > 0 && (ExportOffset < ImportOffset || ImportEntrySize < 28))
		{
			Error = "invalid UE import table";
			return false;
		}
		PackageData.Imports.reserve(static_cast<size_t>(ImportCount));
		for (int32_t Index = 0; Index < ImportCount; ++Index)
		{
			const size_t EntryOffset = static_cast<size_t>(ImportOffset) + static_cast<size_t>(Index) * ImportEntrySize;
			Reader Entry(PackageData.Data, EntryOffset, EntryOffset + ImportEntrySize);
			std::string Ignored;
			PackageImport Import;
			int32_t OuterIndex = 0;
			if (!ReadPackageName(Entry, PackageData.Names, Ignored) || !ReadPackageName(Entry, PackageData.Names, Ignored) ||
				!Entry.Read(OuterIndex) || !ReadPackageName(Entry, PackageData.Names, Import.ObjectName))
			{
				Error = Entry.Error();
				return false;
			}
			PackageData.Imports.push_back(std::move(Import));
		}

		const size_t ExportEntrySize = ExportCount > 0 ?
			static_cast<size_t>(DependsOffset - ExportOffset) / static_cast<size_t>(ExportCount) : 0;
		if (ExportCount <= 0 || DependsOffset < ExportOffset || ExportEntrySize < 44)
		{
			Error = "invalid or empty UE export table";
			return false;
		}
		PackageData.Exports.reserve(static_cast<size_t>(ExportCount));
		for (int32_t Index = 0; Index < ExportCount; ++Index)
		{
			const size_t EntryOffset = static_cast<size_t>(ExportOffset) + static_cast<size_t>(Index) * ExportEntrySize;
			Reader Entry(PackageData.Data, EntryOffset, EntryOffset + ExportEntrySize);
			PackageExport Export;
			if (!Entry.Read(Export.ClassIndex) || !Entry.Skip(12) ||
				!ReadPackageName(Entry, PackageData.Names, Export.ObjectName) || !Entry.Skip(4) ||
				!Entry.Read(Export.SerialSize) || !Entry.Read(Export.SerialOffset))
			{
				Error = Entry.Error();
				return false;
			}
			if (Export.SerialSize < 0 || Export.SerialOffset < 0 ||
				static_cast<uint64_t>(Export.SerialOffset) + static_cast<uint64_t>(Export.SerialSize) > PackageData.Data.size())
			{
				Error = "invalid UE export payload range";
				return false;
			}
			PackageData.Exports.push_back(std::move(Export));
		}
		return true;
	}

	std::string ExportClassName(const Package& PackageData, const PackageExport& Export)
	{
		if (Export.ClassIndex < 0)
		{
			const size_t Index = static_cast<size_t>(-Export.ClassIndex - 1);
			return Index < PackageData.Imports.size() ? PackageData.Imports[Index].ObjectName : std::string();
		}
		if (Export.ClassIndex > 0)
		{
			const size_t Index = static_cast<size_t>(Export.ClassIndex - 1);
			return Index < PackageData.Exports.size() ? PackageData.Exports[Index].ObjectName : std::string();
		}
		return std::string();
	}

	uint32_t ReadBigEndian32(const uint8_t* Data)
	{
		return (static_cast<uint32_t>(Data[0]) << 24) | (static_cast<uint32_t>(Data[1]) << 16) |
			(static_cast<uint32_t>(Data[2]) << 8) | static_cast<uint32_t>(Data[3]);
	}

	uint64_t ReadBigEndian64(const uint8_t* Data)
	{
		return (static_cast<uint64_t>(ReadBigEndian32(Data)) << 32) | ReadBigEndian32(Data + 4);
	}

#ifdef _WIN32
	using OodleDecompressFunction = intptr_t(__cdecl*)(const void*, intptr_t, void*, intptr_t, int, int, int,
		void*, intptr_t, void*, void*, void*, intptr_t, int);

	OodleDecompressFunction FindOodleDecompressor()
	{
		static OodleDecompressFunction Function = nullptr;
		static bool Searched = false;
		if (Searched)
		{
			return Function;
		}
		Searched = true;
		std::vector<std::string> Candidates = { "oo2core_9_win64.dll" };
		char Root[4096] = {};
		for (const char* Variable : { "UE5_ROOT", "UE_ROOT" })
		{
			const DWORD Length = GetEnvironmentVariableA(Variable, Root, static_cast<DWORD>(sizeof(Root)));
			if (Length > 0 && Length < sizeof(Root))
			{
				Candidates.emplace_back(std::string(Root) + "\\Engine\\Source\\Programs\\Shared\\EpicGames.Oodle\\Sdk\\2.9.10\\win\\redist\\oo2core_9_win64.dll");
			}
		}
		Candidates.emplace_back("F:\\src\\UE5_S18\\Engine\\Source\\Programs\\Shared\\EpicGames.Oodle\\Sdk\\2.9.10\\win\\redist\\oo2core_9_win64.dll");
		Candidates.emplace_back("F:\\src\\UE5_S18\\Engine\\Binaries\\ThirdParty\\Oodle\\Win64\\oo2core_9_win64.dll");
		for (const std::string& Candidate : Candidates)
		{
			HMODULE Module = LoadLibraryA(Candidate.c_str());
			if (Module != nullptr)
			{
				Function = reinterpret_cast<OodleDecompressFunction>(GetProcAddress(Module, "OodleLZ_Decompress"));
				if (Function != nullptr)
				{
					break;
				}
			}
		}
		return Function;
	}
#endif

	bool DecompressBuffer(const uint8_t* Compressed, size_t CompressedSize, std::vector<uint8_t>& Raw, std::string& Error)
	{
		if (CompressedSize < 64 || ReadBigEndian32(Compressed) != CompressedBufferTag)
		{
			Error = "invalid UE compressed buffer header";
			return false;
		}
		const uint8_t Method = Compressed[8];
		const uint8_t BlockSizeExponent = Compressed[11];
		const uint32_t BlockCount = ReadBigEndian32(Compressed + 12);
		const uint64_t RawSize = ReadBigEndian64(Compressed + 16);
		const uint64_t TotalCompressedSize = ReadBigEndian64(Compressed + 24);
		if (RawSize == 0 || RawSize > std::numeric_limits<size_t>::max() || TotalCompressedSize > CompressedSize)
		{
			Error = "invalid UE compressed buffer size";
			return false;
		}
		Raw.resize(static_cast<size_t>(RawSize));
		if (Method == 0)
		{
			if (64 + RawSize > CompressedSize)
			{
				Error = "truncated uncompressed UE buffer";
				return false;
			}
			memcpy(Raw.data(), Compressed + 64, static_cast<size_t>(RawSize));
			return true;
		}
		if (Method != 3 || BlockCount == 0 || BlockSizeExponent >= 63 || 64ull + 4ull * BlockCount > CompressedSize)
		{
			Error = "unsupported UE compressed buffer method";
			return false;
		}
#ifndef _WIN32
		Error = "Oodle StaticMesh payloads are currently supported on Windows only";
		return false;
#else
		OodleDecompressFunction OodleDecompress = FindOodleDecompressor();
		if (OodleDecompress == nullptr)
		{
			Error = "cannot load UE5.4 oo2core_9_win64.dll; set UE5_ROOT to the UE5 source root";
			return false;
		}
		const uint64_t BlockSize = 1ull << BlockSizeExponent;
		size_t CompressedOffset = 64 + static_cast<size_t>(BlockCount) * 4;
		size_t RawOffset = 0;
		for (uint32_t BlockIndex = 0; BlockIndex < BlockCount; ++BlockIndex)
		{
			const uint32_t CompressedBlockSize = ReadBigEndian32(Compressed + 64 + static_cast<size_t>(BlockIndex) * 4);
			const size_t RawBlockSize = static_cast<size_t>(std::min<uint64_t>(BlockSize, RawSize - RawOffset));
			if (CompressedBlockSize == 0 || CompressedOffset + CompressedBlockSize > CompressedSize)
			{
				Error = "truncated UE compressed buffer block";
				return false;
			}
			if (CompressedBlockSize < RawBlockSize)
			{
				const intptr_t Result = OodleDecompress(Compressed + CompressedOffset, CompressedBlockSize,
					Raw.data() + RawOffset, RawBlockSize, 1, 1, 0, nullptr, 0, nullptr, nullptr, nullptr, 0, 0);
				if (Result != static_cast<intptr_t>(RawBlockSize))
				{
					Error = "Oodle failed to decompress a UE StaticMesh payload block";
					return false;
				}
			}
			else
			{
				memcpy(Raw.data() + RawOffset, Compressed + CompressedOffset, RawBlockSize);
			}
			CompressedOffset += CompressedBlockSize;
			RawOffset += RawBlockSize;
		}
		return RawOffset == Raw.size();
#endif
	}

	struct TrailerEntry
	{
		std::array<uint8_t, 20> Identifier = {};
		int64_t Offset = 0;
		int64_t CompressedSize = 0;
		int64_t RawSize = 0;
		uint8_t AccessMode = 0;
	};

	bool LoadTrailerPayload(const Package& PackageData, const std::array<uint8_t, 20>& Identifier,
		std::vector<uint8_t>& Payload, std::string& Error)
	{
		if (PackageData.Data.size() < 20)
		{
			Error = "UE package has no package trailer";
			return false;
		}
		Reader Footer(PackageData.Data, PackageData.Data.size() - 20);
		uint64_t FooterTag = 0;
		uint64_t TrailerLength = 0;
		uint32_t PackageTag = 0;
		if (!Footer.Read(FooterTag) || !Footer.Read(TrailerLength) || !Footer.Read(PackageTag) ||
			FooterTag != PackageTrailerFooterTag || PackageTag != UEPackageTag ||
			TrailerLength > PackageData.Data.size())
		{
			Error = "invalid or missing UE package trailer";
			return false;
		}
		const size_t TrailerOffset = PackageData.Data.size() - static_cast<size_t>(TrailerLength);
		Reader Header(PackageData.Data, TrailerOffset, PackageData.Data.size() - 20);
		uint64_t HeaderTag = 0;
		int32_t Version = 0;
		int32_t HeaderLength = 0;
		int64_t PayloadDataLength = 0;
		int32_t EntryCount = 0;
		if (!Header.Read(HeaderTag) || !Header.Read(Version) || !Header.Read(HeaderLength) ||
			!Header.Read(PayloadDataLength) || !Header.Read(EntryCount) || HeaderTag != PackageTrailerHeaderTag ||
			HeaderLength < 28 || EntryCount < 0 || static_cast<int64_t>(HeaderLength) + PayloadDataLength + 20 != static_cast<int64_t>(TrailerLength))
		{
			Error = "invalid UE package trailer header";
			return false;
		}
		if (Version != 2)
		{
			Error = "unsupported UE package trailer version";
			return false;
		}
		for (int32_t Index = 0; Index < EntryCount; ++Index)
		{
			TrailerEntry Entry;
			uint32_t Flags = 0;
			if (!Header.ReadBytes(Entry.Identifier.data(), Entry.Identifier.size()) || !Header.Read(Entry.Offset) ||
				!Header.Read(Entry.CompressedSize) || !Header.Read(Entry.RawSize) || !Header.Read(Flags) || !Header.Read(Entry.AccessMode))
			{
				Error = Header.Error();
				return false;
			}
			if (Entry.Identifier != Identifier)
			{
				continue;
			}
			if (Entry.AccessMode != 0)
			{
				Error = "StaticMesh payload is referenced or virtualized instead of stored in the package";
				return false;
			}
			if (Entry.Offset < 0 || Entry.CompressedSize <= 0 || Entry.Offset + Entry.CompressedSize > PayloadDataLength)
			{
				Error = "invalid StaticMesh package trailer payload range";
				return false;
			}
			const size_t PayloadOffset = TrailerOffset + static_cast<size_t>(HeaderLength) + static_cast<size_t>(Entry.Offset);
			return DecompressBuffer(PackageData.Data.data() + PayloadOffset, static_cast<size_t>(Entry.CompressedSize), Payload, Error);
		}
		Error = "StaticMesh payload identifier was not found in the package trailer";
		return false;
	}

	bool ReadMeshValue(Reader& Input, uint32_t Type)
	{
		switch (Type)
		{
		case 0: return Input.Skip(16);
		case 1: return Input.Skip(12);
		case 2: return Input.Skip(8);
		case 3:
		case 4:
		case 5: return Input.Skip(4);
		case 6:
		{
			std::string Name;
			return Input.ReadString(Name);
		}
		case 7: return Input.Skip(80);
		default: return Input.Fail("unsupported MeshDescription attribute type");
		}
	}

	struct MeshAttribute
	{
		uint32_t Type = 0;
		uint32_t Extent = 0;
		int32_t Count = 0;
		std::vector<uint8_t> Data;
	};

	struct MeshElement
	{
		int32_t BitCount = 0;
		std::vector<uint32_t> ValidBits;
		std::unordered_map<std::string, MeshAttribute> Attributes;
	};

	bool ParseMeshAttributes(Reader& Input, const std::string& ElementName, MeshElement& Element)
	{
		int32_t NumElements = 0;
		int32_t AttributeCount = 0;
		if (!Input.Read(NumElements) || !Input.Read(AttributeCount) || NumElements < 0 || AttributeCount < 0 || AttributeCount > 1024)
		{
			return Input.Fail("invalid MeshDescription attribute map");
		}
		for (int32_t AttributeIndex = 0; AttributeIndex < AttributeCount; ++AttributeIndex)
		{
			std::string AttributeName;
			uint32_t Type = 0;
			uint32_t Extent = 0;
			int32_t StoredNumElements = 0;
			int32_t ChannelCount = 0;
			if (!Input.ReadString(AttributeName) || !Input.Read(Type) || !Input.Read(Extent) ||
				!Input.Read(StoredNumElements) || !Input.Read(ChannelCount) || Type > 7 || Extent == 0 ||
				StoredNumElements < 0 || ChannelCount < 0 || ChannelCount > 128)
			{
				return Input.Fail("invalid MeshDescription attribute entry");
			}
			while (!AttributeName.empty() && (AttributeName.back() == ' ' || AttributeName.back() == '\t'))
			{
				AttributeName.pop_back();
			}
			MeshAttribute Captured;
			Captured.Type = Type;
			Captured.Extent = Extent;
			for (int32_t ChannelIndex = 0; ChannelIndex < ChannelCount; ++ChannelIndex)
			{
				uint32_t ArrayExtent = 0;
				int32_t Count = 0;
				if (!Input.Read(ArrayExtent))
				{
					return false;
				}
				const bool IsBulkType = Type <= 5;
				int32_t ElementSize = 0;
				if (IsBulkType && !Input.Read(ElementSize))
				{
					return false;
				}
				if (!Input.Read(Count) || Count < 0)
				{
					return Input.Fail("invalid MeshDescription attribute array size");
				}
				if (IsBulkType)
				{
					const int32_t ExpectedSize[] = { 16, 12, 8, 4, 4, 1 };
					if (ElementSize != ExpectedSize[Type] || static_cast<uint64_t>(ElementSize) * static_cast<uint64_t>(Count) > Input.Remaining())
					{
						return Input.Fail("invalid MeshDescription bulk attribute data");
					}
					const size_t ByteCount = static_cast<size_t>(ElementSize) * static_cast<size_t>(Count);
					const bool Capture = ChannelIndex == 0 &&
						((ElementName == "Vertices" && AttributeName == "Position") ||
						 (ElementName == "VertexInstances" && AttributeName == "VertexIndex") ||
						 (ElementName == "Triangles" && AttributeName == "VertexInstanceIndex"));
					if (Capture)
					{
						Captured.Count = Count;
						Captured.Data.resize(ByteCount);
						if (!Input.ReadBytes(Captured.Data.data(), ByteCount))
						{
							return false;
						}
					}
					else if (!Input.Skip(ByteCount))
					{
						return false;
					}
				}
				else
				{
					for (int32_t ValueIndex = 0; ValueIndex < Count; ++ValueIndex)
					{
						if (!ReadMeshValue(Input, Type))
						{
							return false;
						}
					}
				}
			}
			if (!ReadMeshValue(Input, Type))
			{
				return false;
			}
			uint32_t Flags = 0;
			if (!Input.Read(Flags))
			{
				return false;
			}
			if (!Captured.Data.empty())
			{
				Element.Attributes.emplace(std::move(AttributeName), std::move(Captured));
			}
		}
		return true;
	}

	bool IsElementValid(const MeshElement& Element, int32_t Index)
	{
		return Index >= 0 && Index < Element.BitCount &&
			(Element.ValidBits[static_cast<size_t>(Index) / 32] & (1u << (static_cast<uint32_t>(Index) & 31))) != 0;
	}

	bool ParseMeshDescription(const std::vector<uint8_t>& Payload, UE5TriangleMeshData& Mesh, std::string& Error)
	{
		Reader Input(Payload);
		int32_t ElementTypeCount = 0;
		if (!Input.Read(ElementTypeCount) || ElementTypeCount <= 0 || ElementTypeCount > 64)
		{
			Error = "invalid MeshDescription element map";
			return false;
		}
		std::unordered_map<std::string, MeshElement> Elements;
		for (int32_t TypeIndex = 0; TypeIndex < ElementTypeCount; ++TypeIndex)
		{
			std::string ElementName;
			int32_t ChannelCount = 0;
			if (!Input.ReadString(ElementName) || !Input.Read(ChannelCount) || ChannelCount <= 0 || ChannelCount > 16)
			{
				Error = Input.Error().empty() ? "invalid MeshDescription element channels" : Input.Error();
				return false;
			}
			for (int32_t ChannelIndex = 0; ChannelIndex < ChannelCount; ++ChannelIndex)
			{
				MeshElement Element;
				int32_t NumHoles = 0;
				if (!Input.Read(Element.BitCount) || Element.BitCount < 0 || Element.BitCount > 100000000)
				{
					Error = "invalid MeshDescription element bit array";
					return false;
				}
				Element.ValidBits.resize((static_cast<size_t>(Element.BitCount) + 31) / 32);
				if (!Input.ReadBytes(Element.ValidBits.data(), Element.ValidBits.size() * sizeof(uint32_t)) ||
					!Input.Read(NumHoles) || !ParseMeshAttributes(Input, ElementName, Element))
				{
					Error = Input.Error();
					return false;
				}
				if (ChannelIndex == 0)
				{
					Elements.emplace(ElementName, std::move(Element));
				}
			}
		}

		const auto VerticesIt = Elements.find("Vertices");
		const auto InstancesIt = Elements.find("VertexInstances");
		const auto TrianglesIt = Elements.find("Triangles");
		if (VerticesIt == Elements.end() || InstancesIt == Elements.end() || TrianglesIt == Elements.end())
		{
			Error = "MeshDescription is missing vertices, vertex instances, or triangles";
			return false;
		}
		const auto PositionsIt = VerticesIt->second.Attributes.find("Position");
		const auto VertexIndicesIt = InstancesIt->second.Attributes.find("VertexIndex");
		const auto TriangleIndicesIt = TrianglesIt->second.Attributes.find("VertexInstanceIndex");
		if (PositionsIt == VerticesIt->second.Attributes.end() || VertexIndicesIt == InstancesIt->second.Attributes.end() ||
			TriangleIndicesIt == TrianglesIt->second.Attributes.end() || PositionsIt->second.Type != 1 ||
			VertexIndicesIt->second.Type != 4 || TriangleIndicesIt->second.Type != 4 || TriangleIndicesIt->second.Extent != 3)
		{
			std::ostringstream Stream;
			Stream << "MeshDescription has unsupported core mesh attributes (position="
				<< (PositionsIt != VerticesIt->second.Attributes.end()) << ", vertex-index="
				<< (VertexIndicesIt != InstancesIt->second.Attributes.end()) << ", triangle-index="
				<< (TriangleIndicesIt != TrianglesIt->second.Attributes.end());
			if (TriangleIndicesIt != TrianglesIt->second.Attributes.end())
			{
				Stream << ", triangle-type=" << TriangleIndicesIt->second.Type << ", extent=" << TriangleIndicesIt->second.Extent;
			}
			Stream << ", vertex-attributes=";
			for (const auto& Attribute : VerticesIt->second.Attributes)
			{
				Stream << '[' << Attribute.first << ']';
			}
			Stream << ")";
			Error = Stream.str();
			return false;
		}

		const MeshAttribute& Positions = PositionsIt->second;
		const MeshAttribute& VertexIndices = VertexIndicesIt->second;
		const MeshAttribute& TriangleIndices = TriangleIndicesIt->second;
		Mesh.Vertices.resize(static_cast<size_t>(Positions.Count));
		for (int32_t Index = 0; Index < Positions.Count; ++Index)
		{
			float Values[3];
			memcpy(Values, Positions.Data.data() + static_cast<size_t>(Index) * 12, 12);
			Mesh.Vertices[static_cast<size_t>(Index)] = Vector3(Values[0], Values[1], Values[2]);
		}
		const int32_t TriangleArraySize = TriangleIndices.Count / 3;
		Mesh.Indices.reserve(static_cast<size_t>(TriangleArraySize) * 3);
		for (int32_t TriangleIndex = 0; TriangleIndex < TriangleArraySize; ++TriangleIndex)
		{
			if (!IsElementValid(TrianglesIt->second, TriangleIndex))
			{
				continue;
			}
			uint32_t Output[3];
			for (int32_t Corner = 0; Corner < 3; ++Corner)
			{
				int32_t VertexInstance = 0;
				memcpy(&VertexInstance, TriangleIndices.Data.data() + (static_cast<size_t>(TriangleIndex) * 3 + Corner) * 4, 4);
				if (VertexInstance < 0 || VertexInstance >= VertexIndices.Count || !IsElementValid(InstancesIt->second, VertexInstance))
				{
					Error = "MeshDescription triangle references an invalid vertex instance";
					return false;
				}
				int32_t VertexIndex = 0;
				memcpy(&VertexIndex, VertexIndices.Data.data() + static_cast<size_t>(VertexInstance) * 4, 4);
				if (VertexIndex < 0 || static_cast<size_t>(VertexIndex) >= Mesh.Vertices.size())
				{
					Error = "MeshDescription vertex instance references an invalid vertex";
					return false;
				}
				Output[Corner] = static_cast<uint32_t>(VertexIndex);
			}
			Mesh.Indices.insert(Mesh.Indices.end(), Output, Output + 3);
		}
		return !Mesh.Vertices.empty() && !Mesh.Indices.empty();
	}

	struct PropertyTag
	{
		std::string Name;
		std::vector<std::string> TypeNames;
		int32_t Size = 0;
		uint8_t Flags = 0;
		size_t ValueOffset = 0;
		size_t ValueEnd = 0;
	};

	bool ReadPropertyTag(Reader& Input, const Package& PackageData, PropertyTag& Tag, bool& IsEnd)
	{
		IsEnd = false;
		if (!ReadPackageName(Input, PackageData.Names, Tag.Name))
		{
			return false;
		}
		if (Tag.Name == "None")
		{
			IsEnd = true;
			return true;
		}
		int32_t RemainingTypeNodes = 1;
		while (RemainingTypeNodes > 0)
		{
			std::string TypeName;
			int32_t InnerCount = 0;
			if (!ReadPackageName(Input, PackageData.Names, TypeName) || !Input.Read(InnerCount) || InnerCount < 0 || InnerCount > 64)
			{
				return Input.Fail("invalid UE property type name");
			}
			Tag.TypeNames.push_back(std::move(TypeName));
			RemainingTypeNodes += InnerCount - 1;
			if (Tag.TypeNames.size() > 128)
			{
				return Input.Fail("UE property type name is too deep");
			}
		}
		if (!Input.Read(Tag.Size) || !Input.Read(Tag.Flags) || Tag.Size < 0)
		{
			return Input.Fail("invalid UE property tag size");
		}
		if ((Tag.Flags & 0x01) != 0 && !Input.Skip(4))
		{
			return false;
		}
		if ((Tag.Flags & 0x02) != 0 && !Input.Skip(16))
		{
			return false;
		}
		if ((Tag.Flags & 0x04) != 0)
		{
			uint8_t ExtensionFlags = 0;
			if (!Input.Read(ExtensionFlags))
			{
				return false;
			}
			if ((ExtensionFlags & 0x02) != 0 && !Input.Skip(2))
			{
				return false;
			}
		}
		Tag.ValueOffset = Input.Position();
		Tag.ValueEnd = Tag.ValueOffset + static_cast<size_t>(Tag.Size);
		if (static_cast<size_t>(Tag.Size) > Input.Remaining())
		{
			return Input.Fail("UE property value exceeds its export");
		}
		return true;
	}

	bool ReadFloatProperty(const std::vector<uint8_t>& Data, const PropertyTag& Tag, float& Value)
	{
		if (Tag.Size != 4)
		{
			return false;
		}
		memcpy(&Value, Data.data() + Tag.ValueOffset, 4);
		return true;
	}

	bool ReadVectorProperty(const std::vector<uint8_t>& Data, const PropertyTag& Tag, Vector3& Value)
	{
		if (Tag.Size == 24)
		{
			double Values[3];
			memcpy(Values, Data.data() + Tag.ValueOffset, sizeof(Values));
			Value = Vector3(static_cast<float>(Values[0]), static_cast<float>(Values[1]), static_cast<float>(Values[2]));
			return true;
		}
		if (Tag.Size == 12)
		{
			float Values[3];
			memcpy(Values, Data.data() + Tag.ValueOffset, sizeof(Values));
			Value = Vector3(Values[0], Values[1], Values[2]);
			return true;
		}
		return false;
	}

	Quaternion UEQuaternionFromRotator(const Vector3& RotatorDegrees)
	{
		const double HalfDegreesToRadians = 3.14159265358979323846 / 360.0;
		const double SP = sin(static_cast<double>(RotatorDegrees.x) * HalfDegreesToRadians);
		const double CP = cos(static_cast<double>(RotatorDegrees.x) * HalfDegreesToRadians);
		const double SY = sin(static_cast<double>(RotatorDegrees.y) * HalfDegreesToRadians);
		const double CY = cos(static_cast<double>(RotatorDegrees.y) * HalfDegreesToRadians);
		const double SR = sin(static_cast<double>(RotatorDegrees.z) * HalfDegreesToRadians);
		const double CR = cos(static_cast<double>(RotatorDegrees.z) * HalfDegreesToRadians);
		return Quaternion(
			static_cast<float>(CR * SP * SY - SR * CP * CY),
			static_cast<float>(-CR * SP * CY - SR * CP * SY),
			static_cast<float>(CR * CP * SY - SR * SP * CY),
			static_cast<float>(CR * CP * CY + SR * SP * SY));
	}

	bool ReadVectorArray(Reader& Value, int32_t PropertySize, std::vector<Vector3>& Output)
	{
		const size_t Start = Value.Position();
		int32_t Count = 0;
		if (!Value.Read(Count) || Count < 0)
		{
			return Value.Fail("invalid UE vector array count");
		}
		const size_t BytesAfterCount = static_cast<size_t>(PropertySize) - 4;
		const size_t ElementSize = Count > 0 ? BytesAfterCount / static_cast<size_t>(Count) : 24;
		if ((Count > 0 && ElementSize != 12 && ElementSize != 24) || static_cast<size_t>(Count) * ElementSize != BytesAfterCount)
		{
			return Value.Fail("invalid UE vector array size");
		}
		Output.resize(static_cast<size_t>(Count));
		for (int32_t Index = 0; Index < Count; ++Index)
		{
			if (ElementSize == 24)
			{
				double Components[3];
				if (!Value.ReadBytes(Components, sizeof(Components)))
				{
					return false;
				}
				Output[static_cast<size_t>(Index)] = Vector3(static_cast<float>(Components[0]), static_cast<float>(Components[1]), static_cast<float>(Components[2]));
			}
			else
			{
				float Components[3];
				if (!Value.ReadBytes(Components, sizeof(Components)))
				{
					return false;
				}
				Output[static_cast<size_t>(Index)] = Vector3(Components[0], Components[1], Components[2]);
			}
		}
		return Value.Position() == Start + static_cast<size_t>(PropertySize);
	}

	bool ReadIndexArray(Reader& Value, int32_t PropertySize, std::vector<uint32_t>& Output)
	{
		const size_t Start = Value.Position();
		int32_t Count = 0;
		if (!Value.Read(Count) || Count < 0 || 4ull + static_cast<uint64_t>(Count) * 4 != static_cast<uint64_t>(PropertySize))
		{
			return Value.Fail("invalid UE index array size");
		}
		Output.resize(static_cast<size_t>(Count));
		for (int32_t Index = 0; Index < Count; ++Index)
		{
			int32_t ValueIndex = 0;
			if (!Value.Read(ValueIndex) || ValueIndex < 0)
			{
				return Value.Fail("invalid negative UE collision index");
			}
			Output[static_cast<size_t>(Index)] = static_cast<uint32_t>(ValueIndex);
		}
		return Value.Position() == Start + static_cast<size_t>(PropertySize);
	}

	bool ReadTransformProperty(Reader& Value, const PropertyTag& Tag, Transform& Output)
	{
		if (Tag.Size != 80)
		{
			return false;
		}
		double Rotation[4];
		double Translation[3];
		double Scale[3];
		if (!Value.ReadBytes(Rotation, sizeof(Rotation)) || !Value.ReadBytes(Translation, sizeof(Translation)) ||
			!Value.ReadBytes(Scale, sizeof(Scale)))
		{
			return false;
		}
		Output.pos = Vector3(static_cast<float>(Translation[0]), static_cast<float>(Translation[1]), static_cast<float>(Translation[2]));
		Output.quat = Quaternion(static_cast<float>(Rotation[0]), static_cast<float>(Rotation[1]),
			static_cast<float>(Rotation[2]), static_cast<float>(Rotation[3]));
		return true;
	}

	bool ParseCollisionElement(const Package& PackageData, Reader& Input, size_t ElementEnd,
		UE5SimpleCollisionType Type, UE5SimpleCollisionData& Collision, std::string& Error)
	{
		Collision.Type = Type;
		Vector3 RotationDegrees = Vector3::Zero();
		float FullX = 0.0f;
		float FullY = 0.0f;
		float FullZ = 0.0f;
		float FullLength = 0.0f;
		while (Input.Position() < ElementEnd)
		{
			PropertyTag Tag;
			bool IsEnd = false;
			if (!ReadPropertyTag(Input, PackageData, Tag, IsEnd))
			{
				Error = Input.Error();
				return false;
			}
			if (IsEnd)
			{
				break;
			}
			Reader Value(PackageData.Data, Tag.ValueOffset, Tag.ValueEnd);
			if (Tag.Name == "Center")
			{
				ReadVectorProperty(PackageData.Data, Tag, Collision.LocalTransform.pos);
			}
			else if (Tag.Name == "Rotation")
			{
				ReadVectorProperty(PackageData.Data, Tag, RotationDegrees);
			}
			else if (Tag.Name == "Radius")
			{
				ReadFloatProperty(PackageData.Data, Tag, Collision.Radius);
			}
			else if (Tag.Name == "Length")
			{
				ReadFloatProperty(PackageData.Data, Tag, FullLength);
			}
			else if (Tag.Name == "X")
			{
				ReadFloatProperty(PackageData.Data, Tag, FullX);
			}
			else if (Tag.Name == "Y")
			{
				ReadFloatProperty(PackageData.Data, Tag, FullY);
			}
			else if (Tag.Name == "Z")
			{
				ReadFloatProperty(PackageData.Data, Tag, FullZ);
			}
			else if (Tag.Name == "VertexData")
			{
				if (!ReadVectorArray(Value, Tag.Size, Collision.Convex.Vertices))
				{
					Error = Value.Error();
					return false;
				}
			}
			else if (Tag.Name == "IndexData")
			{
				if (!ReadIndexArray(Value, Tag.Size, Collision.Convex.Indices))
				{
					Error = Value.Error();
					return false;
				}
			}
			else if (Tag.Name == "Transform")
			{
				ReadTransformProperty(Value, Tag, Collision.LocalTransform);
			}
			if (!Input.Seek(Tag.ValueEnd))
			{
				Error = Input.Error();
				return false;
			}
		}
		if (Type == UE5SimpleCollisionType::Box)
		{
			Collision.HalfExtent = Vector3(FullX * 0.5f, FullY * 0.5f, FullZ * 0.5f);
		}
		else if (Type == UE5SimpleCollisionType::Capsule)
		{
			Collision.HalfHeight = FullLength * 0.5f;
		}
		if (Type == UE5SimpleCollisionType::Box || Type == UE5SimpleCollisionType::Capsule)
		{
			Collision.LocalTransform.quat = UEQuaternionFromRotator(RotationDegrees);
		}
		return true;
	}

	bool ParseCollisionArray(const Package& PackageData, const PropertyTag& ArrayTag, UE5SimpleCollisionType Type,
		std::vector<UE5SimpleCollisionData>& Output, std::string& Error)
	{
		Reader Input(PackageData.Data, ArrayTag.ValueOffset, ArrayTag.ValueEnd);
		int32_t Count = 0;
		if (!Input.Read(Count) || Count < 0 || Count > 100000)
		{
			Error = "invalid UE simple collision array count";
			return false;
		}
		for (int32_t Index = 0; Index < Count; ++Index)
		{
			UE5SimpleCollisionData Collision;
			if (!ParseCollisionElement(PackageData, Input, ArrayTag.ValueEnd, Type, Collision, Error))
			{
				return false;
			}
			Output.push_back(std::move(Collision));
		}
		return true;
	}

	bool ParseAggregateGeometry(const Package& PackageData, const PropertyTag& AggGeomTag,
		std::vector<UE5SimpleCollisionData>& Output, std::string& Error)
	{
		Reader Input(PackageData.Data, AggGeomTag.ValueOffset, AggGeomTag.ValueEnd);
		while (Input.Position() < AggGeomTag.ValueEnd)
		{
			PropertyTag Tag;
			bool IsEnd = false;
			if (!ReadPropertyTag(Input, PackageData, Tag, IsEnd))
			{
				Error = Input.Error();
				return false;
			}
			if (IsEnd)
			{
				return true;
			}
			bool Parsed = true;
			if (Tag.Name == "SphereElems")
			{
				Parsed = ParseCollisionArray(PackageData, Tag, UE5SimpleCollisionType::Sphere, Output, Error);
			}
			else if (Tag.Name == "BoxElems")
			{
				Parsed = ParseCollisionArray(PackageData, Tag, UE5SimpleCollisionType::Box, Output, Error);
			}
			else if (Tag.Name == "SphylElems")
			{
				Parsed = ParseCollisionArray(PackageData, Tag, UE5SimpleCollisionType::Capsule, Output, Error);
			}
			else if (Tag.Name == "ConvexElems")
			{
				Parsed = ParseCollisionArray(PackageData, Tag, UE5SimpleCollisionType::Convex, Output, Error);
			}
			if (!Parsed || !Input.Seek(Tag.ValueEnd))
			{
				if (Error.empty())
				{
					Error = Input.Error();
				}
				return false;
			}
		}
		return true;
	}

	bool ParseSimpleCollision(const Package& PackageData, const PackageExport& BodySetup,
		std::vector<UE5SimpleCollisionData>& Output, std::string& Error)
	{
		const size_t Begin = static_cast<size_t>(BodySetup.SerialOffset);
		const size_t End = Begin + static_cast<size_t>(BodySetup.SerialSize);
		Reader Input(PackageData.Data, Begin, End);
		if (PackageData.UE5Version >= 1011 && !Input.Skip(1))
		{
			Error = Input.Error();
			return false;
		}
		while (Input.Position() < End)
		{
			PropertyTag Tag;
			bool IsEnd = false;
			if (!ReadPropertyTag(Input, PackageData, Tag, IsEnd))
			{
				Error = Input.Error();
				return false;
			}
			if (IsEnd)
			{
				return true;
			}
			if (Tag.Name == "AggGeom" && !ParseAggregateGeometry(PackageData, Tag, Output, Error))
			{
				return false;
			}
			if (!Input.Seek(Tag.ValueEnd))
			{
				Error = Input.Error();
				return false;
			}
		}
		return true;
	}

	bool GetMeshDescriptionPayloadId(const Package& PackageData, const PackageExport& Export,
		std::array<uint8_t, 20>& Identifier, std::string& Error)
	{
		// UE5.4 FMeshDescriptionBulkData ends with FEditorBulkData metadata followed by its GUID fields.
		constexpr size_t SerializedTailSize = 4 + 16 + 20 + 8 + 16 + 4;
		if (Export.SerialSize < static_cast<int64_t>(SerializedTailSize))
		{
			Error = "invalid StaticMeshDescriptionBulkData export";
			return false;
		}
		const size_t FlagsOffset = static_cast<size_t>(Export.SerialOffset + Export.SerialSize) - SerializedTailSize;
		Reader Tail(PackageData.Data, FlagsOffset, static_cast<size_t>(Export.SerialOffset + Export.SerialSize));
		uint32_t Flags = 0;
		std::array<uint8_t, 16> BulkDataId = {};
		int64_t PayloadSize = 0;
		if (!Tail.Read(Flags) || !Tail.ReadBytes(BulkDataId.data(), BulkDataId.size()) ||
			!Tail.ReadBytes(Identifier.data(), Identifier.size()) || !Tail.Read(PayloadSize))
		{
			Error = Tail.Error();
			return false;
		}
		constexpr uint32_t StoredInPackageTrailer = 1u << 9;
		if ((Flags & StoredInPackageTrailer) == 0 || PayloadSize <= 0)
		{
			Error = "StaticMesh MeshDescription payload is not stored in the UE package trailer";
			return false;
		}
		return true;
	}
}

Vector3 UE5ToRiemannVector(const Vector3& Value)
{
	return Vector3(Value.x, Value.z, Value.y);
}

Quaternion UE5ToRiemannQuaternion(const Quaternion& Value)
{
	// A quaternion vector part is an axial vector. Under the improper Y/Z
	// basis swap it transforms as det(S) * S * v, while w is unchanged.
	return Quaternion(-Value.x, -Value.z, -Value.y, Value.w);
}

void ConvertUE5TriangleMeshToRiemann(UE5TriangleMeshData* Mesh)
{
	if (Mesh == nullptr)
	{
		return;
	}
	for (Vector3& Vertex : Mesh->Vertices)
	{
		Vertex = UE5ToRiemannVector(Vertex);
	}
	for (size_t Index = 0; Index + 2 < Mesh->Indices.size(); Index += 3)
	{
		std::swap(Mesh->Indices[Index + 1], Mesh->Indices[Index + 2]);
	}
}

UE5StaticMeshData LoadUE5StaticMesh(const char* Filename)
{
	UE5StaticMeshData Result;
	if (Filename == nullptr || Filename[0] == '\0')
	{
		Result.Error = "StaticMesh filename is empty";
		return Result;
	}

	Package PackageData;
	if (!ParsePackage(Filename, PackageData, Result.Error))
	{
		return Result;
	}
	if (PackageData.UE5Version < 1012)
	{
		Result.Error = "the package predates the UE5.4 StaticMesh property format";
		return Result;
	}

	const PackageExport* StaticMeshExport = nullptr;
	const PackageExport* BodySetupExport = nullptr;
	std::vector<const PackageExport*> LODExports;
	for (const PackageExport& Export : PackageData.Exports)
	{
		const std::string ClassName = ExportClassName(PackageData, Export);
		if (ClassName == "StaticMesh")
		{
			StaticMeshExport = &Export;
		}
		else if (ClassName == "BodySetup")
		{
			BodySetupExport = &Export;
		}
		else if (ClassName == "StaticMeshDescriptionBulkData" && Export.ObjectName != "HiResMeshDescription")
		{
			LODExports.push_back(&Export);
		}
	}
	if (StaticMeshExport == nullptr)
	{
		Result.Error = "the UE package does not contain a StaticMesh export";
		return Result;
	}
	if (LODExports.empty())
	{
		Result.Error = "the StaticMesh has no editor MeshDescription LOD payloads; cooked-only meshes are not supported";
		return Result;
	}

	Result.LODs.reserve(LODExports.size());
	for (const PackageExport* LODExport : LODExports)
	{
		std::array<uint8_t, 20> PayloadIdentifier = {};
		std::vector<uint8_t> Payload;
		UE5TriangleMeshData LOD;
		if (!GetMeshDescriptionPayloadId(PackageData, *LODExport, PayloadIdentifier, Result.Error) ||
			!LoadTrailerPayload(PackageData, PayloadIdentifier, Payload, Result.Error) ||
			!ParseMeshDescription(Payload, LOD, Result.Error))
		{
			return Result;
		}
		Result.LODs.push_back(std::move(LOD));
	}

	if (BodySetupExport != nullptr && !ParseSimpleCollision(PackageData, *BodySetupExport, Result.SimpleCollision, Result.Error))
	{
		return Result;
	}

	for (UE5TriangleMeshData& LOD : Result.LODs)
	{
		ConvertUE5TriangleMeshToRiemann(&LOD);
	}
	for (UE5SimpleCollisionData& Collision : Result.SimpleCollision)
	{
		Collision.LocalTransform.pos = UE5ToRiemannVector(Collision.LocalTransform.pos);
		Collision.LocalTransform.quat = UE5ToRiemannQuaternion(Collision.LocalTransform.quat);
		Collision.HalfExtent = UE5ToRiemannVector(Collision.HalfExtent);
		ConvertUE5TriangleMeshToRiemann(&Collision.Convex);
	}

	// UE uses the render triangle mesh as complex collision when no custom complex mesh is assigned.
	Result.ComplexCollision = Result.LODs.front();
	Result.Success = true;
	return Result;
}
}
