#pragma once

#include <string>
#include <vector>
#include "../Maths/Box3d.h"
#include "../Maths/Vector2.h"
#include "../Maths/Vector3.h"
#include "../Maths/Transform.h"

namespace Geometry
{
	struct GeometryData
	{
		bool LoadObj(const char* name);
		bool ExportObj(const char* name);
		static GeometryData* CreateFromObj(const char* name);

		bool HasVertexColors() const
		{
			return !VertexColors.empty();
		}

		bool HasVertexNormals() const
		{
			return !VertexNormals.empty();
		}

		bool HasVertexUVs() const
		{
			return !VertexUVs.empty();
		}

		std::vector<Vector3>	Vertices;
		std::vector<Vector3>	VertexNormals;
		std::vector<Vector3>	VertexColors;
		std::vector<Vector2>	VertexUVs;
		std::vector<Vector3i>	Triangles;
		Box3d					Bounds;
		Transform				Pose;
		std::string				ResourceName;
	};

	class GeometrySet
	{
	public:
		GeometrySet();
		~GeometrySet();

		bool LoadObj(const char* name, const Transform &pose);

	public:
		std::vector<GeometryData*>	mMeshs;
		Box3d						mBounds;
	};


}	// namespace Destruction
