#include "ComputeDevice.h"

namespace Riemann
{
	namespace SnowCompute
	{
		void Reserve(Device& Device, SharedBuffer& Buffer, std::size_t Bytes)
		{ if (!Buffer || Buffer->Size < Bytes) Buffer = Device.CreateBuffer(Bytes); }
#if !defined(RIEMANN_SNOW_COMPUTE)
		std::unique_ptr<Device> CreateDevice() { return {}; }
#endif
	}
}
