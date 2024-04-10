#pragma once

#include <vector>
#include "../Maths/Box3.h"
#include "../Maths/Transform.h"

namespace Geometry
{
	class DynamicMesh;
}

namespace Destruction
{
	class GeometrySet
	{
	public:
		GeometrySet();
		~GeometrySet();

		bool LoadObj(const char* name, const Transform& pose);
		void CreateGeometryData(int NumMeshes);

	public:
		std::vector<Geometry::DynamicMesh*>	mMeshs;
		Box3									mBounds;
	};
}	// namespace Destruction
