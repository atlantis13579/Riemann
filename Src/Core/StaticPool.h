#pragma once

#include <mutex>

template<typename T, int Capacity>
class StaticPool
{
public:
	StaticPool() : size(0) {}

	T* Get()
	{
		if (size >= Capacity)
		{
			return nullptr;
		}
		return &data[size++];
	}

private:
	T	data[Capacity];
	int	size;
};

template<typename T, int Capacity>
class ThreadSafeStaticPool
{
public:
	ThreadSafeStaticPool() : size(0) {}

	T* Get()
	{
		T* p = nullptr;
		lock.lock();
		if (size < Capacity)
		{
			p = &data[size++];
		}
		lock.unlock();
		return p;
	}

private:
	T	data[Capacity];
	int	size;
	std::mutex	lock;
};
