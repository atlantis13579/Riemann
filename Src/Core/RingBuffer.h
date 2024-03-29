#pragma once

#include <mutex>

template <typename T, int Capacity>
class RingBuffer
{
public:
	RingBuffer()
	{
		Clear();
	}

	inline bool Full() const
	{
		int next = (write + 1) % Capacity;
		return next == read;
	}
	
	inline bool Clear() const
	{
		read = write = 0;
	}

	inline bool Empty() const
	{
		return read == write;
	}

	inline bool Push(const T& v)
	{
		int next = (write + 1) % Capacity;
		if (next != read)
		{
			data[write] = v;
			write = next;
			return true;
		}
		return false;
	}

	inline T Pop()
	{
		if (read != write)
		{
			T item = data[read];
			read = (read + 1) % Capacity;
			return item;
		}
		return T();
	}

private:
	T			data[Capacity];
	int			write;
	int			read;
};

template <typename T, int Capacity>
class ThreadSafeRingBuffer
{
public:
	ThreadSafeRingBuffer()
	{
		Clear();
	}

	void	Clear()
	{
		lock.lock();
		write = read = 0;
		lock.unlock();
	}

	int		GetSize()
	{
		lock.lock();
		int n = (write + Capacity - read) % Capacity;
		lock.unlock();
		return n;
	}

	bool	Push(const T& v)
	{
		bool succ = false;
		lock.lock();
		int next = (write + 1) % Capacity;
		if (next != read)
		{
			data[write] = v;
			write = next;
			succ = true;
		}
		lock.unlock();
		return succ;
	}

	T		Pop()
	{
		T ret = T(0);
		lock.lock();
		if (read != write)
		{
			ret = data[read];
			read = (read + 1) % Capacity;
		}
		lock.unlock();
		return ret;
	}

private:
	T			data[Capacity];
	int			write;
	int			read;
	std::mutex	lock;
};

template <typename T, int Capacity>
class LockFreeRingBuffer
{
};
