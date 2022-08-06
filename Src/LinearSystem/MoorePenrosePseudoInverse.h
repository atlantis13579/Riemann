#pragma once

// Compute the Moore-Penrose Pseudo Inverse of a Matrix
// pinvM = (M^T * M)^(-1) * M^T

template<typename T>
class MoorePenrosePseudoInverse
{
public:
	bool operator()(const T* M, int nDim, T* pinvM) const
	{
	}
};

