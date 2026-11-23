#include "SparseJointSpaceDynamics.h"
#include <cmath>
#include <stdexcept>

namespace Riemann
{
	void SparseJointSpaceDynamics::FactorMassMatrix()
	{
		if (m_FactorRevision == m_ConfigurationRevision)
			return;
		const int Count = GetDofCount();
		const bool BuildPattern = m_Row.empty();
		if (BuildPattern)
			m_Row.resize(Count + 1);
		size_t Next = 0;
		VisitMassTriangle([&](int Row, int Column, double Value)
		{
			if (BuildPattern)
			{
				if (Row == Column)
					m_Row[Row] = Next;
				m_Column.push_back(Column);
				m_Factor.push_back(Value);
			}
			else
				m_Factor[Next] = Value;
			++Next;
		});
		m_Row[Count] = Next;
		std::vector<double> Scale(Count);
		for (int I = 0; I < Count; ++I) Scale[I] = m_Factor[m_Row[I]];
		// A row begins with its diagonal followed by ancestors, nearest first.
		// Each ancestor row has exactly the same column pattern as the suffix.
		for (int I = Count - 1; I >= 0; --I)
		{
			const size_t Begin = m_Row[I], End = m_Row[I + 1];
			const double D = m_Factor[Begin];
			if (!std::isfinite(D) || D <= m_Settings.RelativePivotTolerance * Scale[I] || D <= 0)
				throw std::domain_error("Singular articulated mass matrix");
			for (size_t J = Begin + 1; J < End; ++J)
			{
				const double L = m_Factor[J] / D;
				size_t Target = m_Row[m_Column[J]];
				for (size_t K = J; K < End; ++K) m_Factor[Target++] -= L * m_Factor[K];
				m_Factor[J] = L;
			}
			m_Factor[Begin] = 1.0 / D;
		}
		m_FactorRevision = m_ConfigurationRevision;
		++m_FactorizationCount;
	}

	std::vector<double> SparseJointSpaceDynamics::SolveMass(std::vector<double> Force)
	{
		FactorMassMatrix();
		for (int I = GetDofCount() - 1; I >= 0; --I)
			for (size_t J = m_Row[I] + 1; J < m_Row[I + 1]; ++J)
				Force[m_Column[J]] -= m_Factor[J] * Force[I];
		for (int I = 0; I < GetDofCount(); ++I)
		{
			Force[I] *= m_Factor[m_Row[I]];
			for (size_t J = m_Row[I] + 1; J < m_Row[I + 1]; ++J)
				Force[I] -= m_Factor[J] * Force[m_Column[J]];
		}
		ValidateVector(Force);
		return Force;
	}

	std::vector<double> SparseJointSpaceDynamics::ComputeAcceleration(const std::vector<double>& Torque, const Vector3& Gravity,
		const std::vector<SpatialVector>& ExternalWrenches)
	{
		ValidateVector(Torque);
		std::vector<double> Bias = InverseDynamics(std::vector<double>(Torque.size()), Gravity, ExternalWrenches);
		for (size_t I = 0; I < Torque.size(); ++I) Bias[I] = Torque[I] - Bias[I];
		return SolveMass(std::move(Bias));
	}

	std::vector<double> SparseJointSpaceDynamics::ComputeImpulseResponse(const std::vector<SpatialVector>& LinkImpulses,
		const std::vector<double>& JointImpulses)
	{
		std::vector<double> Force = ComputeGeneralizedForce(LinkImpulses);
		if (!JointImpulses.empty())
		{
			ValidateVector(JointImpulses);
			for (size_t I = 0; I < Force.size(); ++I) Force[I] += JointImpulses[I];
		}
		return SolveMass(std::move(Force));
	}
}
