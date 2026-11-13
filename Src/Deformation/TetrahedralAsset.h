#pragma once

#include <memory>
#include <vector>
#include "TetrahedralMaterial.h"
#include "../Maths/Index3.h"

namespace Riemann
{
	struct TetrahedralAssetElement
	{
		std::array<int, 4> Vertices = {};
		int Material = 0;
	};
	struct TetrahedralAssetInput
	{
		std::vector<Vector3> Positions;
		std::vector<TetrahedralAssetElement> Elements;
		std::vector<TetrahedralMaterial> Materials = { TetrahedralMaterial() };
		std::vector<int> PinnedVertices;
		std::vector<Vector3> RenderPositions;
		std::vector<Index3> RenderTriangles;
	};
	struct TetrahedralAssetSettings
	{
		TetrahedralSolverSettings ElementSettings;
		double MinimumQuality = 1e-5, MinimumEdgeLength = 1e-7;
		double RelativeOverlapTolerance = 1e-8, BarycentricTolerance = 1e-6;
		double RelativeRenderAreaTolerance = 1e-8;
		bool AllowExtrapolation = false;
		double MaximumExtrapolationRatio = 0.25, MaximumExtrapolationWeight = 4;
		bool IsValid() const;
	};
	struct TetrahedralRenderBinding
	{
		int Element = -1;
		std::array<double, 4> Weights = {};
	};
	struct TetrahedralAssetStats
	{
		size_t ReorientedElements = 0, ConnectedComponents = 0, ExtrapolatedVertices = 0;
		double TotalVolume = 0, TotalMass = 0, MinimumQuality = 1;
	};
	class TetrahedralAsset
	{
	public:
		static TetrahedralAsset Prepare(const TetrahedralAssetInput& Input, const TetrahedralAssetSettings& Settings = TetrahedralAssetSettings());
		bool IsValid() const { return m_Data != nullptr; }
		const std::vector<Vector3>& GetPositions() const;
		const std::vector<TetrahedralAssetElement>& GetElements() const;
		const std::vector<TetrahedralMaterial>& GetMaterials() const;
		const std::vector<TetrahedralRestShape>& GetRestShapes() const;
		const std::vector<std::array<int, 2>>& GetEdges() const;
		const std::vector<Index3>& GetBoundary() const;
		const std::vector<float>& GetInverseMasses() const;
		const std::vector<bool>& GetPins() const;
		const std::vector<Index3>& GetRenderTriangles() const;
		const std::vector<Vector3>& GetRenderPositions() const;
		const std::vector<TetrahedralRenderBinding>& GetRenderBindings() const;
		const TetrahedralAssetStats& GetStats() const;
		const TetrahedralAssetSettings& GetSettings() const;
		std::vector<TetrahedralRenderBinding> BindPoints(const std::vector<Vector3>& Points) const;
	private:
		struct Storage;
		const Storage& Data() const;
		std::shared_ptr<const Storage> m_Data;
	};
}
