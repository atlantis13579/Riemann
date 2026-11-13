#pragma once

#include <stdint.h>
#include <vector>

#include "../Solver/SolverBackend.h"

namespace Riemann
{
	class Geometry;
	class ContactManifold;
	class PositionConstraint;
	struct ContactPersistenceSettings;
	struct FrictionPatchSettings;

	struct SequentialImpulseSettings
	{
		int PositionIterations = 8;
		int VelocityIterations = 1;
		bool IsValid() const { return PositionIterations > 0 && VelocityIterations > 0; }
	};

	class ConstraintSolver
	{
	public:
		virtual ~ConstraintSolver() {}
		virtual void BeginFrame(uint64_t FrameId) { (void)FrameId; }
		virtual void ResetState() {}
		virtual void SetFrictionPatchSettings(const FrictionPatchSettings& Settings) = 0;
		virtual void SetWarmStartingEnabled(bool Enabled) { (void)Enabled; }
		virtual int GetWarmStartContactCount() const { return 0; }
		virtual void PreResolve(const std::vector<Geometry*>& Geometries, const std::vector<PositionConstraint*>& Joints = {}) = 0;
		virtual void ResolveContact(const std::vector<Geometry*>& Geometries,
			std::vector<ContactManifold*>& Manifolds, float Dt) = 0;
		virtual void ResolveConstraints(const std::vector<Geometry*>& Geometries, std::vector<ContactManifold*>& Manifolds,
			const std::vector<PositionConstraint*>& Joints, float Dt, int Iterations) = 0;
		virtual void PostResolve(const std::vector<Geometry*>& Geometries) = 0;
		virtual SolverBackend GetBackend() const { return SolverBackend::CPU; }

		static ConstraintSolver* CreateSequentialImpulseSolver(const ContactPersistenceSettings* Settings = nullptr,
			const SequentialImpulseSettings* Iterations = nullptr);
	};
}
