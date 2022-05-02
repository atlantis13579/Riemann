#include <math.h>
#include <memory.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <unordered_map>

#include "AnimBinaryParser.h"
#include "Serialization.h"

const std::uint32_t BIN_SECTION_MAGIC = 0x42a14e65;
// const std::uint32_t ALIGNED_BIN_SECTION_MAGIC = 0x4142a14e;

#define ANIMATION_MAGIC_NUM         "CHAR::ANIM"
#define ANIMATION_HEADER            "HEADER"
#define ANIMATION_CHANNEL_DATA      "CHANNEL_DATA"
#define ANIMATION_MOTION_DATA       "MOTION_DATA"
#define ANIMATION_TRACKS_DATA       "TRACKS_DATA"
#define ANIMATION_GAIT_DATA         "GAIT_DATA"

#pragma pack(push,1)
// define head format, total 32 byte
struct AnimationHeader
{
	char magicNum[10];  // "CHAR::ANIM"
	std::uint16_t version;  // update to 2
	std::uint16_t numOfBone;
	std::uint16_t midFlag; //not be used, fill 0
	int duration;
	std::uint8_t compressType;  // 	AC_LOW_QUALITY = 0,AC_MID_QUALITY = 1,AC_HIGH_QUALITY = 2,AC_LOSSLESS = 3,AC_ACL = 4
	std::uint8_t flag;
	int createTime;
	std::uint8_t quantBitWidth;  //value is : 12(5byte),15(6byte),20(8byte) ,  and 0==12
	std::uint8_t frameRate;  //ACL frameRate
	float maxError; // fill 0
};
#pragma pack( pop )

template <class Keysequence>
static size_t GetKeySequenceSizeBinCompress(char* buf, const Keysequence& keysequence)
{
	uint32_t keyFrameCount = *reinterpret_cast<uint32_t*>(buf);
	return keyFrameCount * sizeof(typename Keysequence::value_type);
}

static std::pair<uint32_t, bool> decodeLongsequenceInfo(const uint32_t val)
{
	if (val & 0x80000000) // ‭10000000000000000000000000000000‬
	{
		return std::make_pair(val & 0x7FFFFFFF, true);
	}
	else
		return std::make_pair(val, false);
}


static void SetNByetInt(uint64_t& dst, int n, char* src)
{
	dst = 0;
	for (int idx = 0; idx < n; ++idx)
	{
		dst |= uint64_t(uint8_t(src[idx])) << (idx * 8);
	}
}

static void decodeRotKeyFrameInfo(std::uint8_t code, bool& isLongTime, bool& isQuant, int& bitwidth)
{
	isLongTime = bool(code & 0x1);
	isQuant = (code & 0x2) != 0;
	if (isQuant)
	{
		bitwidth = code >> 2;
	}
}

static int bitwidth_to_charcount(int bitwidth)
{
	int charcount = 5; // 12bit
	if (bitwidth == 15)
		charcount = 6;
	else if (bitwidth == 20)
		charcount = 8;
	return charcount;
}


// used by version_1
static std::pair<uint32_t, bool> decodeHighLevelQuantizeinfo(const uint32_t val)
{
	if (val & 0x40000000) // 0x40000000 = ‭01000000000000000000000000000000‬
	{
		return std::make_pair(val & 0xBFFFFFFF, true);  //0xBFFFFFFF = 10111111111111111111111111111111
	}
	else
		return std::make_pair(val, false);
}

template <class ValueType>
size_t GetKeySequenceSizeBinCompress(char* buf, const std::vector<std::pair<int, ValueType>>& keysequence)
{
	uint32_t keyFrameCount = *reinterpret_cast<uint32_t*>(buf);
	auto res = decodeLongsequenceInfo(keyFrameCount);
	keyFrameCount = res.first;
	int indexSize = res.second ? 3 : 2;
	return keyFrameCount * (indexSize + sizeof(ValueType));
}

template <>
size_t GetKeySequenceSizeBinCompress<uint64_t>(char* buf, const std::vector<std::pair<int, uint64_t>>& keysequence)
{
	uint32_t keyFrameCount = *reinterpret_cast<uint32_t*>(buf);
	auto res = decodeLongsequenceInfo(keyFrameCount);
	keyFrameCount = res.first;
	int indexSize = res.second ? 3 : 2;

	res = decodeHighLevelQuantizeinfo(keyFrameCount);
	keyFrameCount = res.first;
	int quantizeSize = res.second ? 6 : 5;

	return keyFrameCount * (indexSize + quantizeSize);
}

template <class KeyFramesType>
size_t computeKeyFramesBufSize(char* buf, KeyFramesType& keyFrames, bool enableBinaryCompress)
{
	if (!enableBinaryCompress)
	{
		uint32_t keyFrameCount = *reinterpret_cast<uint32_t*>(buf);
		return keyFrameCount * sizeof(typename KeyFramesType::value_type);
	}
	else
	{
		return GetKeySequenceSizeBinCompress(buf, keyFrames);
	}
}

uint32_t ReadKeySequenceBinCompress(char* buf, std::vector< std::pair<int, uint64_t>>& sequence)
{
	uint32_t ret = 0;
	sequence.clear();
	uint32_t keyFrameCount = *reinterpret_cast<uint32_t*>(buf);
	buf += sizeof(uint32_t);
	ret += sizeof(uint32_t);

	auto res = decodeLongsequenceInfo(keyFrameCount);
	keyFrameCount = res.first;
	bool isLongSeq = res.second;
	int indexSize = isLongSeq ? 3 : 2;

	res = decodeHighLevelQuantizeinfo(keyFrameCount);
	keyFrameCount = res.first;
	bool isHighLevel = res.second;
	int quantizeSize = isHighLevel ? 6 : 5;

	for (uint32_t idx = 0; idx < keyFrameCount; ++idx)
	{
		uint64_t index = 0;
		SetNByetInt(index, indexSize, buf);
		buf += indexSize;
		ret += indexSize;

		uint64_t value;
		SetNByetInt(value, quantizeSize, buf);
		buf += quantizeSize;
		ret += quantizeSize;

		sequence.emplace_back(int(index), value);
	}

	return ret;
}

template <class ValueType>
static uint32_t ReadKeySequenceBinCompress(char* buf, std::vector< std::pair<int, ValueType>>& sequence)
{
	uint32_t ret = 0;
	sequence.clear();
	uint32_t keyFrameCount = *reinterpret_cast<uint32_t*>(buf);
	buf += sizeof(uint32_t);
	ret += sizeof(uint32_t);
	auto res = decodeLongsequenceInfo(keyFrameCount);
	keyFrameCount = res.first;
	bool isLongSeq = res.second;
	int indexSize = isLongSeq ? 3 : 2;

	for (uint32_t idx = 0; idx < keyFrameCount; ++idx)
	{
		uint64_t index = 0;
		SetNByetInt(index, indexSize, buf);
		buf += indexSize;
		ret += indexSize;

		ValueType value;
		memcpy(&value, buf, sizeof(ValueType));
		buf += sizeof(ValueType);
		ret += sizeof(ValueType);
		sequence.emplace_back(int(index), value);
	}

	return ret;
}

template <class KeyFramesType>
static size_t readKeyFramesFromBuf(char* buf, KeyFramesType& keyFrames, bool enableBinaryCompress)
{
	if (!enableBinaryCompress)
	{
		uint32_t keyFrameCount = *reinterpret_cast<uint32_t*>(buf);
		buf += sizeof(uint32_t);
		auto frameBuf = reinterpret_cast<typename KeyFramesType::value_type*>(buf);
		keyFrames.assign(frameBuf, frameBuf + keyFrameCount);
		return sizeof(uint32_t) + keyFrameCount * sizeof(typename KeyFramesType::value_type);
	}
	else
	{
		return ReadKeySequenceBinCompress(buf, keyFrames);
	}
}

template <class Keysequence>
static uint32_t ReadKeySequenceBinCompress(char* buf, Keysequence& sequence)
{
	uint32_t keyFrameCount = *reinterpret_cast<uint32_t*>(buf);
	buf += sizeof(uint32_t);
	auto frameBuf = reinterpret_cast<typename Keysequence::value_type*>(buf);
	sequence.assign(frameBuf, frameBuf + keyFrameCount);
	return sizeof(uint32_t) + keyFrameCount * sizeof(typename Keysequence::value_type);
}

template <typename INTTYPE>
static void Decode(INTTYPE val, float& x, float& y, float& z, float& w, int BITLENGTH)
{
	const float startV = 1.0f / sqrtf(2.0f);
	const INTTYPE maxNum = INTTYPE(powf(2.0f, float(BITLENGTH)) - 1);
	const float maxV = startV * float(maxNum);

	INTTYPE idx = (val >> 3 * BITLENGTH) & 3;
	if (idx == 0)
	{
		y = ((val >> 2 * BITLENGTH) & maxNum) / maxV - startV;
		z = ((val >> BITLENGTH) & maxNum) / maxV - startV;
		w = (val & maxNum) / maxV - startV;
		x = sqrtf(1.0f - y * y - z * z - w * w);
	}
	else if (idx == 1)
	{
		z = ((val >> 2 * BITLENGTH) & maxNum) / maxV - startV;
		w = ((val >> BITLENGTH) & maxNum) / maxV - startV;
		x = (val & maxNum) / maxV - startV;
		y = sqrtf(1.0f - x * x - z * z - w * w);
	}
	else if (idx == 2)
	{
		w = ((val >> 2 * BITLENGTH) & maxNum) / maxV - startV;
		x = ((val >> BITLENGTH) & maxNum) / maxV - startV;
		y = (val & maxNum) / maxV - startV;
		z = sqrtf(1.0f - y * y - x * x - w * w);
	}
	else
	{
		x = ((val >> 2 * BITLENGTH) & maxNum) / maxV - startV;
		y = ((val >> BITLENGTH) & maxNum) / maxV - startV;
		z = (val & maxNum) / maxV - startV;
		w = sqrtf(1.0f - y * y - z * z - x * x);
	}
}

static void dequantizer(const std::vector< std::pair<int, uint64_t>>& keyframes, RotationKeyframes& result, int bitwidth)
{
	result.reserve(keyframes.size());
	for (uint32_t i = 0; i < keyframes.size(); ++i)
	{
		float x = 0.f, y = 0.f, z = 0.f, w = 0.f;
		Decode<uint64_t>(keyframes[i].second, x, y, z, w, bitwidth);

		Quaternion q(x, y, z, w);
		result.emplace_back(std::pair<int, Quaternion>(keyframes[i].first, q));
	}
}

struct BinaryBlock
{
	char* data;
	int len;
};

class AnimationSerializer : public AnimTreeData
{
public:
	AnimationSerializer()
	{
		memset(&header, 0, sizeof(header));
	}

	virtual ~AnimationSerializer()
	{

	}

	int DeserializeFile(const char* filepath)
	{
		FILE* fp = fopen(filepath, "rb");
		if (!fp)
			return -1;
		fseek(fp, 0, SEEK_END);
		uint64_t fileSize = ftell(fp);
		fseek(fp, 0, SEEK_SET);
		std::vector<char> buffer;
		buffer.resize((size_t)fileSize);
		fread(&buffer[0], 1, (size_t)fileSize, fp);
		fclose(fp);
		int ret = DeserializeImpl(&buffer[0], (int)buffer.size());
		return ret;
	}

private:
	int DeserializeImpl(char* data, int len)
	{
		ReadBinaryIndex(data, len);
		if (sections.empty())
		{
			return 1;
		}

		if (sections.find(ANIMATION_HEADER) == sections.end() || sections[ANIMATION_HEADER].len < (int)sizeof(AnimationHeader))
		{
			return 2;
		}
		header = read<AnimationHeader>(sections[ANIMATION_HEADER].data);
		if (memcmp(header.magicNum, ANIMATION_MAGIC_NUM, sizeof(ANIMATION_MAGIC_NUM) - 1) != 0)
		{
			return 3;
		}

		int ret = 0;
		if (header.version == 2)
		{
			ret = DeserializeBoneChannels();
			if (ret != 0)
				return ret;
		}

		ret = DeserializeMotionChannels();
		if (ret != 0)
			return ret;

		return 0;
	}

	void ReadBinaryIndex(char* data, int len)
	{
		int entryDataOffset = 0;
		int entryNameLenPad = 0;
		if (*(std::uint32_t*)data == BIN_SECTION_MAGIC)
		{
			entryDataOffset = 4;
			entryNameLenPad = 3;
		}

		// read the index info
		int indexLen = *(int*)(data + len - sizeof(int));
		int offset = len - sizeof(int) - indexLen;

		// make sure it is valid
		if (offset < entryDataOffset || offset >= len - int(sizeof(int)))
		{
			return;
		}

		// read the directory out of the file
		while (offset <= len - (int)(sizeof(int) + sizeof(int) * 2))
		{
			// read the lengths of directory entry
			int entryDataLen = *(int*)(data + offset);
			offset += sizeof(int);
			if (((entryDataLen & (1U << 31)) || entryNameLenPad != 0) &&
				offset + sizeof(std::uint32_t) * 4 <= len - (sizeof(int) + sizeof(int)))
			{
				// skip extended data
				entryDataLen &= ~(1U << 31);
				// uint32 preloadLen, uint32 version, uint64 modified
				offset += sizeof(std::uint32_t) * 4;
			}
			int entryNameLen = *(int*)(data + offset);
			offset += sizeof(int);

			// make sure they make sense
			if (std::uint32_t(entryDataLen) > 256 * 1024 * 1028 ||
				std::uint32_t(entryDataOffset + entryDataLen) >
				std::uint32_t(len - sizeof(int) - indexLen) ||
				std::uint32_t(entryNameLen) > 4096 ||
				std::uint32_t(offset + entryNameLen) > std::uint32_t(len - sizeof(int)))
			{
				sections.clear();
				return;
			}

			// read its name
			std::string entryStr(data + offset, entryNameLen);
			offset += (entryNameLen + entryNameLenPad) & (~entryNameLenPad);

			// add it to our list of children
			BinaryBlock block;
			block.data = data + entryDataOffset;
			block.len = entryDataLen;
			sections[entryStr] = block;

			// move on the data offset
			entryDataOffset += (entryDataLen + 3) & (~3L);
		}
	}

	static uint32_t readRotKeyFrameFromBuf(char* buf, RotationKeyframes& rotateKeys)
	{
		uint32_t ret = 0;
		rotateKeys.clear();
		std::uint16_t keyFrameCount = *reinterpret_cast<uint16_t*>(buf);
		buf += sizeof(std::uint16_t);
		ret += sizeof(std::uint16_t);

		if (keyFrameCount == 0)
			return ret;

		std::uint8_t datainfo = *reinterpret_cast<uint8_t*>(buf);
		buf += sizeof(std::uint8_t);
		ret += sizeof(std::uint8_t);

		bool isLongSeq = false;
		bool isQuant = true;
		int bitwidth = 12;
		decodeRotKeyFrameInfo(datainfo, isLongSeq, isQuant, bitwidth);
		int indexSize = isLongSeq ? 3 : 2;
		if (isQuant)
		{
			int quantizeSize = bitwidth_to_charcount(bitwidth);
			std::vector< std::pair<int, uint64_t>> sequence;
			sequence.reserve(keyFrameCount);
			for (uint32_t idx = 0; idx < keyFrameCount; ++idx)
			{
				uint64_t index = 0;
				SetNByetInt(index, indexSize, buf);
				buf += indexSize;
				ret += indexSize;
				uint64_t value;
				SetNByetInt(value, quantizeSize, buf);
				buf += quantizeSize;
				ret += quantizeSize;
				sequence.emplace_back(int(index), value);
			}
			dequantizer(sequence, rotateKeys, bitwidth);

		}
		else
		{
			rotateKeys.reserve(keyFrameCount);
			for (uint32_t idx = 0; idx < keyFrameCount; ++idx)
			{
				uint64_t index = 0;
				SetNByetInt(index, indexSize, buf);
				buf += indexSize;
				ret += indexSize;
				Quaternion value = *reinterpret_cast<Quaternion*>(buf);
				buf += sizeof(Quaternion);
				ret += sizeof(Quaternion);
				rotateKeys.emplace_back(std::make_pair(int(index), value));
			}
		}
		return ret;
	}


	int DeserializeBoneChannels()
	{
		if (sections.find(ANIMATION_CHANNEL_DATA) == sections.end())
		{
			return 11;
		}

		char* buf = sections[ANIMATION_CHANNEL_DATA].data;
		const int bufSize = sections[ANIMATION_CHANNEL_DATA].len;

		// use to count the expected buffer size.
		int bufCount = 0;

		std::vector<BoneChannel>& boneChannels = channels;
		// TODO: use resize or use swap?

		int boneCount = header.numOfBone;
		boneChannels.resize(boneCount);

		for (int i = 0; i < boneCount; ++i)
		{
			// check the buf size
			if (bufCount > bufSize) break;
			// read bone name length
			std::uint8_t len = *reinterpret_cast<std::uint8_t*>(buf);
			buf += sizeof(std::uint8_t);
			bufCount += sizeof(std::uint8_t);

			// check the buf size
			if (bufCount > bufSize) break;
			// read bone name
			boneChannels[i].boneName = std::string(buf, len);
			buf += sizeof(char) * len;
			bufCount += sizeof(char) * len;

			//read parent pos in channels
			std::int16_t parentPos = *reinterpret_cast<std::int16_t*>(buf);
			boneChannels[i].parentPos = int(parentPos);
			buf += sizeof(std::int16_t);
			bufCount += sizeof(std::int16_t);

			if (hasScaleKeyframes)
			{
				// check the buf size
				bufCount += sizeof(uint32_t);
				if (bufCount > bufSize) break;
				bufCount += int(computeKeyFramesBufSize(buf, boneChannels[i].scaleKeys, true));
				if (bufCount > bufSize) break;
				// read the scale key frames
				buf += int(readKeyFramesFromBuf(buf, boneChannels[i].scaleKeys, true));
			}

			// check the buf size
			bufCount += sizeof(uint32_t);
			if (bufCount > bufSize) break;
			bufCount += int(computeKeyFramesBufSize(buf, boneChannels[i].positionKeys, true));
			if (bufCount > bufSize) break;
			// read the position key frames
			buf += int(readKeyFramesFromBuf(buf, boneChannels[i].positionKeys, true));

			// read rotation keyframes;
			uint32_t rotsize = readRotKeyFrameFromBuf(buf, boneChannels[i].rotationKeys);
			bufCount += rotsize;
			if (bufCount > bufSize) break;
			buf += rotsize;
		}

		if (bufCount > bufSize)
		{
			return 12;
		}

		return 0;
	}

	int DeserializeMotionChannels()
	{
		if (sections.find(ANIMATION_MOTION_DATA) == sections.end())
		{
			return 21;
		}

		char* buf = sections[ANIMATION_MOTION_DATA].data;
		const int bufSize = sections[ANIMATION_MOTION_DATA].len;

		MotionChannel& mc = motion;
		bool isBinaryCompress = header.version >= 2;
		// use to count the expected buffer size.
		int bufCount = 0;
		do
		{
			// read motion frames
			bufCount += sizeof(uint32_t);
			if (bufCount > bufSize) break;
			bufCount += int(computeKeyFramesBufSize(buf, mc.positionKeys, isBinaryCompress));
			if (bufCount > bufSize) break;
			buf += readKeyFramesFromBuf(buf, mc.positionKeys, isBinaryCompress);

			bufCount += sizeof(uint32_t);
			if (bufCount > bufSize) break;
			bufCount += int(computeKeyFramesBufSize(buf, mc.yawKeys, isBinaryCompress));
			if (bufCount > bufSize) break;
			buf += readKeyFramesFromBuf(buf, mc.yawKeys, isBinaryCompress);

		} while (false);

		if (bufCount > bufSize)
		{
			return 22;
		}

		return 0;
	}
private:
	AnimationHeader header;
	bool hasScaleKeyframes = false;
	std::unordered_map<std::string, BinaryBlock> sections;
};

AnimTreeData* AnimTreeData::Deserialize(const char* filepath)
{
	AnimationSerializer* ptr = new AnimationSerializer;
	if (ptr->DeserializeFile(filepath) != 0)
	{
		delete ptr;
		ptr = nullptr;
	}
	return ptr;
}
