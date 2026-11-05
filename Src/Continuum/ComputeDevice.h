#pragma once

#include <memory>
#include <string>

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

namespace Riemann
{
namespace SnowCompute
{
	struct BackendInfo
	{
		bool Available = false;
		std::string API, DeviceName, Message;
	};

	struct Buffer
	{
		virtual ~Buffer() = default;
		std::size_t Size = 0;
	};
	using SharedBuffer = std::shared_ptr<Buffer>;
	using Bindings = std::array<SharedBuffer, 8>;

	struct Program { virtual ~Program() = default; };
	using SharedProgram = std::shared_ptr<Program>;

	struct Constants
	{
		std::array<std::uint32_t, 64> Words{};
		void Integer(std::size_t index, int value) { Words[index] = static_cast<std::uint32_t>(value); }
		void Float(std::size_t index, float value) { std::memcpy(&Words[index], &value, sizeof(value)); }
	};

	struct Readback
	{
		SharedBuffer Source;
		void* Destination;
		std::size_t Bytes;
	};

	class Device
	{
	public:
		virtual ~Device() = default;
		virtual SharedBuffer CreateBuffer(std::size_t bytes) = 0;
		virtual SharedProgram CreateProgram(const char* source, const char* entry) = 0;
		virtual void Upload(const SharedBuffer& buffer, const void* data, std::size_t bytes) = 0;
		virtual void Dispatch(const SharedProgram& program, const Bindings& buffers,
							  const Constants& constants, int threads) = 0;
		virtual void Download(const std::vector<Readback>& requests) = 0;
		// End failed batches before releasing resources or resuming on CPU.
		virtual void Finish() = 0;
		virtual BackendInfo GetInfo() const = 0;
	};

	std::unique_ptr<Device> CreateDevice();
	void Reserve(Device& device, SharedBuffer& buffer, std::size_t bytes);
}
}
