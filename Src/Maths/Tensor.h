#pragma once

#include <memory.h>

template <class T>
struct Rank
{
	static constexpr int value = 0;
};

template <class T, int Dim>
struct Rank<T[Dim]>
{
	static constexpr int value = 1 + Rank<T>::value;
};

template <typename DataType, int Rank>
class Tensor
{
public:
	template<typename... IndexTypes>
	Tensor<DataType, Rank>(IndexTypes ...Dims)
	{
		static_assert(sizeof...(Dims) == Rank, "tensor size is not right");

		pData = nullptr;
		HoldMemory = false;
		_InitTensor(Dims...);

		Size = 1;
		for (int i = 0; i < Rank; ++i)
		{
			Size *= Dimensions[i];
			if (i == 0)
				SubDimensions[Rank - 1] = 1;
			else
				SubDimensions[Rank - i - 1] = SubDimensions[Rank - i] * Dimensions[Rank - i];
		}
		if (Size > 0)
		{
			pData = new DataType[Size];
			HoldMemory = true;
		}
	}

	Tensor<DataType, Rank>()
	{
		pData = nullptr;
		HoldMemory = false;
		Size = 1;
	}

	~Tensor<DataType, Rank>()
	{
		if (pData && HoldMemory)
		{
			delete[]pData;
			pData = nullptr;
		}
	}

	void 	Load(DataType* p)
	{
		memcpy(pData, p, sizeof(DataType) * Size);
	}

	void	LoadZero()
	{
		memset(pData, 0, sizeof(DataType) * Size);
	}
	
	inline constexpr int GetRank() const
	{
		return Rank;
	}

	inline int GetDimension(int Index) const
	{
		return Dimensions[Index];
	}

	inline bool SameDimension(const Tensor<DataType, Rank>& rhs)
	{
		for (int i = 0; i < Rank; ++i)
		{
			if (Dimensions[i] != rhs.Dimensions[i])
			{
				return false;
			}
		}
		return true;
	}

	int     GetSize() const
	{
		return Size;
	}

	inline const Tensor<DataType, Rank>& operator=(const Tensor<DataType, Rank>& rhs)
	{
		if (SameDimension(rhs))
		{
			if (pData)
				Load(rhs.pData);
		}
		return *this;
	}

	template<typename... IndexTypes>
	inline DataType operator()(int Index, IndexTypes ...Indices) const
	{
		static_assert(sizeof...(Indices) + 1 == Rank, "indices size is not right");
		return _At(Index * SubDimensions[0], Indices...);
	}

	template<typename... IndexTypes>
	inline DataType& operator()(int Index, IndexTypes ...Indices)
	{
		static_assert(sizeof...(Indices) + 1 == Rank, "indices size is not right");
		return _At(Index * SubDimensions[0], Indices...);
	}

	template<typename DT, int nRank>
	struct SubTensor
	{
		static Tensor<DT, nRank - 1> View(const Tensor<DT, nRank>& rhs, int Index)
		{
			Tensor<DT, nRank - 1> tensor;
			tensor.Size = rhs.Size / rhs.Dimensions[0];
			for (int i = 0; i < nRank - 1; ++i)
			{
				tensor.Dimensions[i] = rhs.Dimensions[i + 1];
				tensor.SubDimensions[i] = rhs.SubDimensions[i + 1];
			}
			tensor.HoldMemory = false;
			tensor.pData = rhs.pData + Index * rhs.SubDimensions[0];
			return tensor;
		}
	};

	template<typename DT>
	struct SubTensor<DT, 1>
	{
		static DT View(const Tensor<DT, Rank>& rhs, int Index)
		{
			return rhs.pData[Index];
		}
	};

	auto operator[](int Index) const
	{
		return SubTensor<DataType, Rank>::View(*this, Index);
	}

protected:
	template<typename... IndexTypes>
	void    _InitTensor(int Dim, IndexTypes ...OtherDims)
	{
		static_assert(sizeof...(OtherDims) + 1 <= Rank, "tensor size is not right");
		Dimensions[Rank - sizeof...(OtherDims) - 1] = Dim;
		_InitTensor(OtherDims...);
	}

	void    _InitTensor()
	{
	}

	template<typename... IndexTypes>
	DataType   _At(int Base, int Index, IndexTypes ...OtherIndices) const
	{
		static_assert(sizeof...(OtherIndices) + 1 <= Rank, "indices size is not right");
		return _At(Base + SubDimensions[Rank - sizeof...(OtherIndices) - 1] * Index, OtherIndices...);
	}

	DataType   _At(int Base) const
	{
		return pData[Base];
	}

	template<typename... IndexTypes>
	DataType& _At(int Base, int Index, IndexTypes ...OtherIndices)
	{
		static_assert(sizeof...(OtherIndices) + 1 <= Rank, "indices size is not right");
		return _At(Base + SubDimensions[Rank - sizeof...(OtherIndices) - 1] * Index, OtherIndices...);
	}

	DataType& _At(int Base)
	{
		return pData[Base];
	}

public:
	int             Size;
	int             Dimensions[Rank];
	int             SubDimensions[Rank];
	bool            HoldMemory;
	DataType*		pData;
};
