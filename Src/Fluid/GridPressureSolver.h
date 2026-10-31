#pragma once

#include <algorithm>
#include <cmath>
#include <vector>
#include "../Maths/Vector3.h"

namespace Riemann
{
	// Matrix-free, diagonal-preconditioned CG replaces fixed-count Jacobi sweeps.
	// Air is zero-pressure Dirichlet data; container walls are Neumann data.
	// A sentinel eliminates neighbor branches from the repeated seven-point
	// matrix product. MaxIterations is a cap; the infinity-norm residual stops
	// at 1e-4 relative or 1e-5 absolute divergence tolerance.
	// Reference: Bridson and Mueller-Fischer, Fluid Simulation course notes,
	// https://www.cs.ubc.ca/~rbridson/fluidsimulation/fluids_notes.pdf
	// Their MIC(0) preconditioner is not used here.
	class GridPressureSolver
	{
	public:
		void Solve(const std::vector<float>& Density, float ActiveThreshold,
			const std::vector<float>& Divergence, const Vector3i& Resolution,
			const Vector3& CellSize, bool OpenTop, float RightHandSideScale,
			int MaxIterations, std::vector<float>& Pressure)
		{
			const int Count = (int)Density.size();
			m_Rows.clear();
			m_Residual.assign(Count + 1, 0.0f);
			m_Search.assign(Count + 1, 0.0f);
			m_Product.resize(Count + 1);
			std::fill(Pressure.begin(), Pressure.end(), 0.0f);
			const int Stride[3] = { 1, Resolution.x, Resolution.x * Resolution.y };
			const float Weight[3] = { 1.0f / (CellSize.x * CellSize.x),
				1.0f / (CellSize.y * CellSize.y), 1.0f / (CellSize.z * CellSize.z) };
			double Sigma = 0.0;
			float MaxResidual = 0.0f;
			for (int Z = 0; Z < Resolution.z; ++Z)
			for (int Y = 0; Y < Resolution.y; ++Y)
			for (int X = 0; X < Resolution.x; ++X)
			{
				const int Index = X + Y * Stride[1] + Z * Stride[2];
				if (Density[Index] <= ActiveThreshold) continue;
				Row Cell;
				Cell.Index = Index;
				Cell.Diagonal = 0.0f;
				const int Coordinate[3] = { X, Y, Z };
				for (int Axis = 0; Axis < 3; ++Axis)
				for (int Side = 0; Side < 2; ++Side)
				{
					const bool Inside = Side == 0 ? Coordinate[Axis] > 0
						: Coordinate[Axis] + 1 < Resolution[Axis];
					const int Neighbor = Index + (Side == 0 ? -Stride[Axis] : Stride[Axis]);
					Cell.Neighbor[Axis * 2 + Side] = Inside && Density[Neighbor] > ActiveThreshold
						? Neighbor : Count;
					if (Inside || (OpenTop && Axis == 1 && Side == 1)) Cell.Diagonal += Weight[Axis];
				}
				if (!(Cell.Diagonal > 0.0f)) continue;
				Cell.InverseDiagonal = 1.0f / Cell.Diagonal;
				m_Rows.push_back(Cell);
				const float Residual = -RightHandSideScale * Divergence[Index];
				m_Residual[Index] = Residual;
				m_Search[Index] = Residual * Cell.InverseDiagonal;
				Sigma += (double)Residual * m_Search[Index];
				MaxResidual = std::max(MaxResidual, std::fabs(Residual));
			}
			const float Tolerance = std::max(1.0e-4f * MaxResidual,
				1.0e-5f * RightHandSideScale);
			for (int Iteration = 0; Iteration < MaxIterations && MaxResidual > Tolerance; ++Iteration)
			{
				double Denominator = 0.0;
				for (const Row& Cell : m_Rows)
				{
					const int* N = Cell.Neighbor;
					const float Product = Cell.Diagonal * m_Search[Cell.Index]
						- Weight[0] * (m_Search[N[0]] + m_Search[N[1]])
						- Weight[1] * (m_Search[N[2]] + m_Search[N[3]])
						- Weight[2] * (m_Search[N[4]] + m_Search[N[5]]);
					m_Product[Cell.Index] = Product;
					Denominator += (double)m_Search[Cell.Index] * Product;
				}
				if (!(Denominator > 0.0) || !(Sigma > 0.0)) break;
				const float Alpha = (float)(Sigma / Denominator);
				double NextSigma = 0.0;
				MaxResidual = 0.0f;
				for (const Row& Cell : m_Rows)
				{
					const int I = Cell.Index;
					Pressure[I] += Alpha * m_Search[I];
					m_Residual[I] -= Alpha * m_Product[I];
					NextSigma += (double)m_Residual[I] * m_Residual[I] * Cell.InverseDiagonal;
					MaxResidual = std::max(MaxResidual, std::fabs(m_Residual[I]));
				}
				const float Beta = (float)(NextSigma / Sigma);
				for (const Row& Cell : m_Rows)
				{
					const int I = Cell.Index;
					m_Search[I] = m_Residual[I] * Cell.InverseDiagonal + Beta * m_Search[I];
				}
				Sigma = NextSigma;
			}
		}

	private:
		struct Row
		{
			int Index;
			int Neighbor[6];
			float Diagonal;
			float InverseDiagonal;
		};
		std::vector<Row> m_Rows;
		std::vector<float> m_Residual, m_Search, m_Product;
	};
}
