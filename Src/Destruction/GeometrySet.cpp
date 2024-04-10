
#include "GeometrySet.h"
#include "../Geometry/DynamicMesh.h"

namespace Destruction
{
	GeometrySet::GeometrySet()
	{

	}

	GeometrySet::~GeometrySet()
	{
		for (size_t i = 0; i < mMeshs.size(); ++i)
		{
			delete mMeshs[i];
		}
		mMeshs.clear();
	}

	bool GeometrySet::LoadObj(const char* name, const Transform& pose)
	{
		Geometry::DynamicMesh* p = Geometry::DynamicMesh::CreateFromObj(name);
		if (p == nullptr)
		{
			return false;
		}

		p->Pose = pose;

		mMeshs.push_back(p);

		if (mMeshs.size() == 1)
		{
			mBounds = p->Bounds;
		}
		else
		{
			mBounds.Encapsulate(p->Bounds);
		}

		return true;
	}

	void GeometrySet::CreateGeometryData(int NumMeshes)
	{
		mMeshs.resize(NumMeshes);
		for (size_t i = 0; i < mMeshs.size(); ++i)
		{
			mMeshs[i] = new Geometry::DynamicMesh();
		}
	}

}	// namespace Destruction
