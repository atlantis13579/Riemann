
#include "GeometrySet.h"
#include "../Geometry/DynamicMesh.h"

namespace Riemann
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
		DynamicMesh* p = DynamicMesh::CreateFromObj(name);
		if (p == nullptr)
		{
			return false;
		}

		p->SetPose(pose);

		mMeshs.push_back(p);

		if (mMeshs.size() == 1)
		{
			mBounds = p->GetBounds();
		}
		else
		{
			mBounds.Encapsulate(p->GetBounds());
		}

		return true;
	}

	void GeometrySet::CreateGeometryData(int NumMeshes)
	{
		mMeshs.resize(NumMeshes);
		for (size_t i = 0; i < mMeshs.size(); ++i)
		{
			mMeshs[i] = new DynamicMesh();
		}
	}

}	// namespace Riemann
