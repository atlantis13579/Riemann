
#pragma once

template<class T, int N>
class Array
{
public:
	Array()
	{
		sz = 0;
	}

	~Array()
	{
	}

	inline T& operator[](int i) {
		return varr[i];
	}

	inline const T& operator[](int i) const {
		return varr[i];
	}

	inline void push(T v) {
		varr[sz++] = v;
	}

	inline T pop()
	{
		if (sz > 0)
			return varr[--sz];
		else
			return 0;
	}

	inline void reset() {
		sz = 0;
	}

	inline int size() const {
		return sz;
	}

private:
	int sz;
	T varr[N];
};