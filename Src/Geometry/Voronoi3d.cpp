
#include "Voronoi3d.h"
#include "../Maths/Maths.h"

namespace Geometry
{
	void GenerateVoronoiSites(const Box3d& Bounds, int pointsMin, int pointsMax, std::vector<Vector3>& sites)
	{
		const Vector3 Extent(Bounds.Max - Bounds.Min);

		int Count = Maths::RandomInt(pointsMin, pointsMax);

		sites.resize(Count);
		for (int i = 0; i < Count; ++i)
		{
			sites[i] = Bounds.Min + Vector3::Random() * Extent;
		}
	}

	Box3d GetVoronoiBounds(const Box3d& Bounds, const std::vector<Vector3>& sites)
	{
		Box3d VoronoiBounds = Bounds;

		if (sites.size() > 0)
		{
			VoronoiBounds += Box3d(sites.data(), (int)sites.size());
		}

		return VoronoiBounds.Thicken(1e-3f);
	}
}