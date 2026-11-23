#pragma once

#include "../../RigidBodyDynamics/FeatherstoneSolver.h"

namespace Riemann
{
	// CRBA joint-space mass matrix and RNEA bias with a tree-sparse L^T D L solve.
	class SparseJointSpaceDynamics final : public FeatherstoneSolver
	{
	public:
		using FeatherstoneSolver::FeatherstoneSolver;
		std::vector<double> ComputeAcceleration(const std::vector<double>& Torque, const Vector3& Gravity,
			const std::vector<SpatialVector>& ExternalWrenches = {}) override;
		std::vector<double> ComputeImpulseResponse(const std::vector<SpatialVector>& LinkImpulses,
			const std::vector<double>& JointImpulses = {}) override;
	private:
		void FactorMassMatrix();
		std::vector<double> SolveMass(std::vector<double> Force);
		size_t m_FactorRevision = size_t(-1);
		std::vector<size_t> m_Row;
		std::vector<int> m_Column;
		std::vector<double> m_Factor;
	};
}
