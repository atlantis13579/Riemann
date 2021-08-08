#pragma once

#include <stdint.h>
#include <cstddef>

template <typename T>
inline T read(uint8_t*& address)
{
	const T value = *reinterpret_cast<T*>(address);
	address += sizeof(value);
	return value;
}

inline uint8_t read8(uint8_t*& address)
{
	return read<uint8_t>(address);
}

inline uint16_t read16(uint8_t*& address)
{
	return read<uint16_t>(address);
}

inline uint32_t read32(uint8_t*& address)
{
	return read<uint32_t>(address);
}

inline uint64_t read64(uint8_t*& address)
{
	return read<uint64_t>(address);
}

inline  uint32_t getPadding(size_t value, uint32_t alignment)
{
	const uint32_t mask = alignment - 1;
	const uint32_t overhead = uint32_t(value) & mask;
	return (alignment - overhead) & mask;
}

inline uint8_t* alignPtr(uint8_t* ptr, uint32_t alignment = 16)
{
	if (alignment == 0)
	{
		return ptr;
	}
	const uint32_t padding = getPadding(size_t(ptr), alignment);
	return ptr + padding;
}

#define MAKE_FOURCC(a, b, c, d) ( (a) | ((b)<<8) | ((c)<<16) | ((d)<<24) )