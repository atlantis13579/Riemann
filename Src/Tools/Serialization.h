#pragma once

template <typename T>
inline T read(unsigned char*& address)
{
	const T value = *reinterpret_cast<T*>(address);
	address += sizeof(value);
	return value;
}

inline unsigned char read8(unsigned char*& address)
{
	return read<unsigned int>(address);
}

inline unsigned short read16(unsigned char*& address)
{
	return read<unsigned int>(address);
}

inline unsigned int read32(unsigned char*& address)
{
	return read<unsigned int>(address);
}

inline unsigned long long read64(unsigned char*& address)
{
	return read<unsigned long long>(address);
}

inline  unsigned int getPadding(size_t value, unsigned int alignment)
{
	const unsigned int mask = alignment - 1;
	const unsigned int overhead = unsigned int(value) & mask;
	return (alignment - overhead) & mask;
}

inline unsigned char* alignPtr(unsigned char* ptr, unsigned int alignment = 16)
{
	if (alignment == 0)
	{
		return ptr;
	}
	const unsigned int padding = getPadding(size_t(ptr), alignment);
	return ptr + padding;
}

#define MAKE_FOURCC(a, b, c, d) ( (a) | ((b)<<8) | ((c)<<16) | ((d)<<24) )