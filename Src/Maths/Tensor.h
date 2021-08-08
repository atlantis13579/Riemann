#pragma once

#include <memory.h>

template <typename DataType, int Rank>
class Tensor
{
public:
	template<typename... IndexTypes>
	Tensor<DataType, Rank>(IndexTypes ...Dims)
	{
		static_assert(sizeof...(Dims) == Rank, "tensor size is not right");

		m_Data = nullptr;
		n_HoldMemory = false;
		_InitTensor(Dims...);

		m_Size = 1;
		for (int i = 0; i < Rank; ++i)
		{
			m_Size *= m_Dimensions[i];
			if (i == 0)
				m_SubDimensions[Rank - 1] = 1;
			else
				m_SubDimensions[Rank - i - 1] = m_SubDimensions[Rank - i] * m_Dimensions[Rank - i];
		}
		if (m_Size > 0)
		{
			m_Data = new DataType[m_Size];
			n_HoldMemory = true;
		}
	}

	Tensor<DataType, Rank>()
	{
		m_Data = nullptr;
		n_HoldMemory = false;
		m_Size = 1;
	}

	~Tensor<DataType, Rank>()
	{
		if (m_Data && n_HoldMemory)
		{
			delete[]m_Data;
			m_Data = nullptr;
		}
	}

	void Load(DataType* p)
	{
		memcpy(m_Data, p, sizeof(DataType) * m_Size);
	}

	void LoadZero()
	{
		memset(m_Data, 0, sizeof(DataType) * m_Size);
	}

	int  Dimension(int Index) const
	{
		return m_Dimensions[Index];
	}

	inline bool SameDimension(const Tensor<DataType, Rank>& rhs)
	{
		for (int i = 0; i < Rank; ++i)
		{
			if (m_Dimensions[i] != rhs.m_Dimensions[i])
			{
				return false;
			}
		}
		return true;
	}

	int     GetSize() const
	{
		return m_Size;
	}

	inline const Tensor<DataType, Rank>& operator=(const Tensor<DataType, Rank>& rhs)
	{
		if (SameDimension(rhs))
		{
			if (m_Data)
				Load(rhs.m_Data);
		}
		return *this;
	}

	template<typename... IndexTypes>
	inline DataType operator()(int Index, IndexTypes ...Indices) const
	{
		static_assert(sizeof...(Indices) + 1 == Rank, "indices size is not right");
		return _At(Index * m_SubDimensions[0], Indices...);
	}

	template<typename... IndexTypes>
	inline DataType& operator()(int Index, IndexTypes ...Indices)
	{
		static_assert(sizeof...(Indices) + 1 == Rank, "indices size is not right");
		return _At(Index * m_SubDimensions[0], Indices...);
	}

	template<typename DT, int nRank>
	struct SubTensor
	{
		static Tensor<DT, nRank - 1> View(const Tensor<DT, nRank>& rhs, int Index)
		{
			Tensor<DT, nRank - 1> tensor;
			tensor.m_Size = rhs.m_Size / rhs.m_Dimensions[0];
			for (int i = 0; i < nRank - 1; ++i)
			{
				tensor.m_Dimensions[i] = rhs.m_Dimensions[i + 1];
				tensor.m_SubDimensions[i] = rhs.m_SubDimensions[i + 1];
			}
			tensor.n_HoldMemory = false;
			tensor.m_Data = rhs.m_Data + Index * rhs.m_SubDimensions[0];
			return tensor;
		}
	};

	template<typename DT>
	struct SubTensor<DT, 1>
	{
		static DT View(const Tensor<DT, Rank>& rhs, int Index)
		{
			return rhs.m_Data[Index];
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
		m_Dimensions[Rank - sizeof...(OtherDims) - 1] = Dim;
		_InitTensor(OtherDims...);
	}

	void    _InitTensor()
	{
	}

	template<typename... IndexTypes>
	DataType   _At(int Base, int Index, IndexTypes ...OtherIndices) const
	{
		static_assert(sizeof...(OtherIndices) + 1 <= Rank, "indices size is not right");
		return _At(Base + m_SubDimensions[Rank - sizeof...(OtherIndices) - 1] * Index, OtherIndices...);
	}

	DataType   _At(int Base) const
	{
		return m_Data[Base];
	}

	template<typename... IndexTypes>
	DataType& _At(int Base, int Index, IndexTypes ...OtherIndices)
	{
		static_assert(sizeof...(OtherIndices) + 1 <= Rank, "indices size is not right");
		return _At(Base + m_SubDimensions[Rank - sizeof...(OtherIndices) - 1] * Index, OtherIndices...);
	}

	DataType& _At(int Base)
	{
		return m_Data[Base];
	}

protected:
	int             m_Size;
	int             m_Dimensions[Rank];
	int             m_SubDimensions[Rank];
	bool            n_HoldMemory;
	DataType* m_Data;
};
