#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "SignedDistanceField.h"

namespace Riemann
{
	struct RuntimeDistanceFieldSettings
	{
		int LeafCellCount = 8;
		int RayRootIterations = 48;
		int ProjectionIterations = 12;
		uint32_t MaximumSearchNodes = 65536;
		float RelativeValueTolerance = 1.0e-5f;
		float RelativeDistanceTolerance = 0.01f;
		float MinimumGradient = 1.0e-8f;
		bool IsValid() const;
	};

	struct DistanceFieldSample
	{
		float Value = 0.0f;
		Vector3 Gradient = Vector3::Zero();
	};

	struct DistanceFieldQueryStats
	{
		uint64_t NodeVisits = 0;
		uint64_t CellTests = 0;
		uint64_t Subdivisions = 0;
	};

	struct DistanceFieldRayHit
	{
		float Time = 0.0f;
		Vector3 Position = Vector3::Zero();
		Vector3 Normal = Vector3::Zero();
		bool NormalValid = false;
		bool InitialOverlap = false;
		Index3 Cell = Index3(0, 0, 0);
	};

	enum class DistanceFieldSearchStatus { InvalidInput, Complete, WorkLimit, PrecisionLimit };

	struct DistanceFieldSurfacePoint
	{
		Vector3 Position = Vector3::Zero();
		Vector3 Normal = Vector3::Zero();
		float Distance = 0.0f;
		float ErrorBound = 0.0f;
		bool NormalValid = false;
		bool Converged = false;
		DistanceFieldSearchStatus Status = DistanceFieldSearchStatus::InvalidInput;
		Index3 Cell = Index3(0, 0, 0);
	};

	struct DistanceFieldTargetPoint
	{
		Vector3 Point = Vector3::Zero();
		Vector3 Normal = Vector3::Zero();
		float Distance = 0.0f;
		float ErrorBound = 0.0f;
		bool SupportingPlane = false;
	};

	class DistanceFieldDistanceTarget
	{
	public:
		virtual ~DistanceFieldDistanceTarget() = default;
		virtual bool Evaluate(const Vector3& Position, DistanceFieldTargetPoint& Result) const = 0;
		virtual double LowerBound(const Box3& Bounds) const;
		virtual DistanceFieldSearchStatus GetStatus() const { return DistanceFieldSearchStatus::Complete; }
	};

	struct DistanceFieldTargetDistance
	{
		Vector3 Position = Vector3::Zero();
		Vector3 TargetPoint = Vector3::Zero();
		Vector3 Normal = Vector3::Zero();
		Index3 Cell = Index3(0, 0, 0);
		float LowerBound = 0.0f;
		float UpperBound = std::numeric_limits<float>::infinity();
		bool Converged = false;
		DistanceFieldSearchStatus Status = DistanceFieldSearchStatus::InvalidInput;
	};

	class RuntimeDistanceField
	{
	public:
		bool Build(const SignedDistanceField3& Field,
			const RuntimeDistanceFieldSettings& Settings = RuntimeDistanceFieldSettings(), std::string* Error = nullptr);
		void Clear() { m_Data.reset(); }
		bool IsValid() const { return m_Data != nullptr; }
		bool HasClosedBoundary() const;
		const SignedDistanceField3* GetField() const;
		Box3 GetBounds() const;
		float GetCellSize() const;
		float GetGradientBound() const;
		size_t GetNodeCount() const;
		uint64_t GetSurfaceCellCount() const;

		bool Sample(const Vector3& Position, DistanceFieldSample& Result) const;
		bool GetCellRange(const Index3& Cell, float& Minimum, float& Maximum) const;
		void QueryCells(const Box3& Bounds, float MinimumValue, float MaximumValue,
			std::vector<Index3>& Cells, DistanceFieldQueryStats* Stats = nullptr) const;
		bool RayCast(const Vector3& Origin, const Vector3& Direction, float MaxTime, DistanceFieldRayHit& Hit,
			bool InitialOverlap = true, DistanceFieldQueryStats* Stats = nullptr) const;
		bool ClosestSurfacePoint(const Vector3& Position, DistanceFieldSurfacePoint& Result,
			float MaxDistance = std::numeric_limits<float>::max(), DistanceFieldQueryStats* Stats = nullptr, float DistanceTolerance = 0.0f) const;
		bool DistanceToTarget(const DistanceFieldDistanceTarget& Target, DistanceFieldTargetDistance& Result,
			DistanceFieldQueryStats* Stats = nullptr, float DistanceTolerance = 0.0f) const;

	private:
		struct Storage;
		std::shared_ptr<const Storage> m_Data;
	};
}
