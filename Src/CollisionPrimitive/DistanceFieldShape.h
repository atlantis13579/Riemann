#pragma once

#include "PrimitiveType.h"
#include "../Geometry/RuntimeDistanceField.h"

namespace Riemann
{
	class StaticMesh;
	struct DistanceFieldShapeSettings
	{
		int ProjectionIterations = 16;
		int WitnessRefinementIterations = 8;
		int MaximumSweepIterations = 128;
		float PatchNormalCosine = 0.98f;
		float MinimumFacingCosine = 0.5f;
		float ManifoldDepthToleranceCells = 0.01f;
		int MassSamplesPerAxis = 2;
		float RelativeSurfaceTolerance = 1.0e-4f;
		float RelativeSweepTolerance = 0.01f;
		float ProjectionStepCells = 2.0f;
		bool IsValid() const;
	};

	struct DistanceFieldSurfaceSample
	{
		Vector3 Position;
		uint32_t Feature;
	};

	class DistanceFieldShape
	{
	public:
		static constexpr PrimitiveType StaticType() { return PrimitiveType::DISTANCE_FIELD; }
		bool Build(const RuntimeDistanceField& Field,
			const DistanceFieldShapeSettings& Settings = DistanceFieldShapeSettings(), std::string* Error = nullptr);
		bool IsValid() const { return m_Data != nullptr; }
		const RuntimeDistanceField& GetField() const;
		const DistanceFieldShapeSettings& GetSettings() const;
		const StaticMesh& GetDisplayMesh() const;
		float GetSurfaceTolerance() const;
		uint32_t GetSurfaceFeature(const Vector3& Point) const;
		bool Project(const Vector3& Point, Vector3& Surface, Vector3& Normal, float& SignedDistance) const;
		void GetSurfaceSamples(const Box3& Bounds, std::vector<DistanceFieldSurfaceSample>& Samples) const;
		bool IntersectRay(const Vector3& Origin, const Vector3& Direction, float* Time) const;
		bool CalculateVolumeProperties(MassParameters* Properties, float Density) const;
		Vector3 GetCenter() const;
		Vector3 GetSupport(const Vector3& Direction) const;
		int GetSupportFace(const Vector3&, Vector3*) const { return 0; }
	private:
		struct Storage;
		std::shared_ptr<const Storage> m_Data;
	};
}
