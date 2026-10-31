#pragma once

#include <stdint.h>
#include <vector>

namespace Riemann
{
	class Geometry;
	class ContactManifold;

	enum class SolverBackend : uint8_t
	{
		CPU,
		SIMD,
		ComputeShader,
		CUDA,
	};

	class ConstraintSolver
	{
	public:
		virtual ~ConstraintSolver() {}
		virtual void BeginFrame(uint64_t FrameId) { (void)FrameId; }
		virtual void ResetState() {}
		virtual void PreResolve(const std::vector<Geometry*>& Geometries) = 0;
		virtual void ResolveContact(const std::vector<Geometry*>& Geometries,
			std::vector<ContactManifold*>& Manifolds, float Dt) = 0;
		virtual void PostResolve(const std::vector<Geometry*>& Geometries) = 0;
		virtual SolverBackend GetBackend() const { return SolverBackend::CPU; }

		static ConstraintSolver* CreateSequentialImpulseSolver();
	};
}
