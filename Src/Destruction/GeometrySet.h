#pragma once

#include <vector>
#include "../Maths/Box3.h"
#include "../Maths/Transform.h"

namespace Riemann
{
	class DynamicMesh;
}

namespace Riemann
{
	class GeometrySet
	{
	public:
		GeometrySet();
		~GeometrySet();

		bool LoadObj(const char* name, const Transform& pose);
		void CreateGeometryData(int NumMeshes);

	public:
		std::vector<DynamicMesh*>	mMeshs;
		Box3									mBounds;
	};
}	// namespace Riemann
