#pragma once

#include "stdio.h"
#include <vector>

#if defined(__linux__) && !defined(__android__)
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/mman.h>
#endif

namespace Riemann
{
	class IBinaryData
	{
	public:
		virtual ~IBinaryData() {};
		virtual unsigned char* GetData() = 0;
		virtual const unsigned char* GetData() const = 0;
		virtual size_t GetSize() const = 0;
	};

	class MemoryFile : public IBinaryData
	{
	public:
		MemoryFile(const char* filename)
		{
			FILE* file = fopen(filename, "rb");
			if (file)
			{
				fseek(file, 0, SEEK_END);
				size_t size = ftell(file);
				fseek(file, 0, SEEK_SET);

				if (size > 0)
				{
					buffer_.resize(size);
					fread(buffer_.data(), 1, size, file);
				}
				fclose(file);
			}
		}

		virtual ~MemoryFile() override final
		{
		}

		virtual unsigned char* GetData() override final
		{
			return buffer_.data();
		}

		virtual const unsigned char* GetData() const override final
		{
			return buffer_.data();
		}

		virtual size_t GetSize() const override final
		{
			return buffer_.size();
		}

	private:
		std::vector<unsigned char>	buffer_;
	};

	template<int AlignSize>
	class MemoryFileAligned : public IBinaryData
	{
	public:
		MemoryFileAligned(const char* filename, size_t offset = 0) : data_(nullptr), size_(0)
		{
			FILE* file = fopen(filename, "rb");
			if (file)
			{
				fseek(file, 0, SEEK_END);
				size_ = ftell(file);
				fseek(file, 0, SEEK_SET);

				if (size_ > offset)
				{
					size_ = size_ - offset;

					const int AlignSize1 = AlignSize - 1;
					buffer_.resize(size_ + AlignSize1);
					data_ = buffer_.data();
					unsigned char* aligned_data = reinterpret_cast<unsigned char*>(
						(reinterpret_cast<size_t>(data_) + AlignSize1) & ~static_cast<size_t>(AlignSize1)
						);
					fseek(file, (long)offset, SEEK_SET);
					fread(aligned_data, size_, 1, file);
					data_ = aligned_data;
				}
				fclose(file);
			}
		}

		virtual ~MemoryFileAligned() override final
		{
			data_ = nullptr;
			size_ = 0;
			buffer_.clear();
		}

		virtual unsigned char* GetData() override final
		{
			return data_;
		}

		virtual const unsigned char* GetData() const override final
		{
			return data_;
		}

		virtual size_t GetSize() const override final
		{
			return size_;
		}

	private:
		unsigned char* data_;
		size_t size_;

		std::vector<unsigned char>	buffer_;
	};

#if defined(__linux__) && !defined(__android__)
	class MmapFile : public IBinaryData
	{
	public:
		MmapFile(const char* filename) : addr_(nullptr), size_(0), aligned_data_(nullptr)
		{
			int fd = open(Filename, O_RDONLY);
			if (fd == -1)
			{
				return nullptr;
			}

			struct stat st;
			fstat(fd, &st);
			size_ = st.st_size;

			addr_ = mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_PRIVATE, fd, 0);
			if (addr_ == MAP_FAILED)
			{
				close(fd);
				return;
			}

			unsigned char* aligned_data = reinterpret_cast<unsigned char*>(
				(reinterpret_cast<size_t>(addr_) + 127) & ~static_cast<size_t>(127)
				);
			assert(aligned_data == addr_);
			aligned_data_ = aligned_data;

			close(fd);
		}

		virtual ~MemoryFile() override final
		{
			if (addr_)
			{
				munmap(addr_, size_);
				addr_ = nullptr;
			}
			aligned_data_ = nullptr;
			size_ = 0;
		}

		virtual unsigned char* GetData() override final
		{
			return aligned_data_;
		}

		virtual const unsigned char* GetData() const override final
		{
			return aligned_data_;
		}

		virtual size_t GetSize() const override final
		{
			return size_;
		}

	private:
		unsigned char* addr_;
		size_t size_;

		void* aligned_data_;
	};
#endif

	class IFile
	{
	public:
		virtual ~IFile() {};
		virtual bool	IsLoaded() const = 0;
		virtual void	Close() = 0;
		virtual size_t	Tell() const = 0;
		virtual size_t	Read(void *buf, size_t size) = 0;
		virtual size_t	Write(const void* buf, size_t size) = 0;
	};

	class FileReader : public IFile
	{
	public:
		FileReader(const char* filename): fp_(nullptr), size_(0)
		{
			fp_ = fopen(filename, "rb");
			if (fp_)
			{
				fseek(fp_, 0, SEEK_END);
				size_ = ftell(fp_);
				fseek(fp_, 0, SEEK_SET);
			}
		}

		virtual ~FileReader() override final
		{
			Close();
		}

		virtual bool	IsLoaded() const override final
		{
			return fp_ != nullptr;
		}

		virtual void	Close() override final
		{
			if (fp_)
				fclose(fp_);
		}

		virtual size_t	Tell() const override final
		{
			return fp_ ? ftell(fp_) : 0;
		}

		virtual size_t	Read(void* buf, size_t size) override final
		{
			return fp_ ? fread(buf, size, 1, fp_) : 0;
		}

		virtual size_t	Write(const void* buf, size_t size) override final
		{
			return 0;
		}

		void	Seek(size_t offset)
		{
			if (fp_)
				fseek(fp_, (long int)offset, SEEK_SET);
		}

		size_t	GetFileSize() const
		{
			return size_;
		}

	protected:
		FILE	*fp_;
		size_t	size_;
	};

	class FileWriter : public IFile
	{
	public:
		explicit FileWriter(const char* filename) : fp_(nullptr)
		{
			fp_ = fopen(filename, "wb");
		}

		virtual ~FileWriter() override final
		{
			Close();
		}

		virtual bool	IsLoaded() const override final
		{
			return fp_ != nullptr;
		}

		virtual void	Close() override final
		{
			if (fp_)
				fclose(fp_);
		}

		virtual size_t	Tell() const override final
		{
			return fp_ ? ftell(fp_) : 0;
		}

		virtual size_t	Read(void* buf, size_t size) override final
		{
			return 0;
		}

		virtual size_t	Write(const void* buf, size_t size) override final
		{
			if (fp_)
			{
				return fwrite(buf, size, 1, fp_);
			}
			return 0;
		}

	protected:
		FILE*	fp_;
	};
}