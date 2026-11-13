#pragma once

#include "CollidingContact.h"

namespace Riemann
{
	struct MeshContactSettings
	{
		bool Enabled = true;
		bool TwoSidedMeshes = true;
		bool ClosedComponentsAreSolids = true;
		float RelativeTolerance = 1.0e-5f;
		float MinimumTolerance = 1.0e-6f;
		float SmoothNormalCosine = 0.999f;
		float PatchNormalCosine = 0.98f;
		bool IsValid() const;
	};

	class MeshContactGenerator
	{
	public:
		explicit MeshContactGenerator(const MeshContactSettings& Settings = MeshContactSettings());
		~MeshContactGenerator();
		bool IsEnabled() const;
		void SetEnabled(bool Enabled);
		void Reset();
		void Invalidate(const Geometry* Shape);
		void Generate(int IndexA, int IndexB, const GeometryWorldState& A, const GeometryWorldState& B,
			std::vector<ContactManifold>& Manifolds, float ContactDistance = 0.0f, float GeometryTolerance = 0.0f);
	private:
		struct Storage;
		std::unique_ptr<Storage> m_Storage;
	};
}
