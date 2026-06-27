#pragma once

#include <stdint.h>

namespace Riemann
{
	static const uint32_t kInvalidGeometryHandleIndex = UINT32_MAX;

	struct GeometryHandle
	{
		GeometryHandle()
			: Index(kInvalidGeometryHandleIndex)
			, Generation(0)
		{
		}

		GeometryHandle(uint32_t index, uint32_t generation)
			: Index(index)
			, Generation(generation)
		{
		}

		bool IsValid() const
		{
			return Index != kInvalidGeometryHandleIndex;
		}

		bool operator==(const GeometryHandle& rhs) const
		{
			return Index == rhs.Index && Generation == rhs.Generation;
		}

		bool operator!=(const GeometryHandle& rhs) const
		{
			return !(*this == rhs);
		}

		uint32_t Index;
		uint32_t Generation;
	};
}
