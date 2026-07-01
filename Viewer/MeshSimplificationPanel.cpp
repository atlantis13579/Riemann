#include "MeshSimplificationPanel.h"

#include <algorithm>
#include <ctype.h>
#include <sstream>
#include <utility>

namespace Riemann
{
	namespace
	{
		float BoundsMaxDim(const Box3& bounds)
		{
			const Vector3 size = bounds.Max - bounds.Min;
			return std::max(size.x, std::max(size.y, size.z));
		}

		bool HasPathExtension(const std::string& fileName, const char* extension)
		{
			if (extension == nullptr)
			{
				return false;
			}

			std::string ext(extension);
			if (fileName.size() <= ext.size() || fileName[fileName.size() - ext.size() - 1] != '.')
			{
				return false;
			}

			const size_t offset = fileName.size() - ext.size();
			for (size_t i = 0; i < ext.size(); ++i)
			{
				const char a = static_cast<char>(tolower(static_cast<unsigned char>(fileName[offset + i])));
				const char b = static_cast<char>(tolower(static_cast<unsigned char>(ext[i])));
				if (a != b)
				{
					return false;
				}
			}
			return true;
		}

		void RefreshStaticMeshPointers(StaticMesh* mesh)
		{
			if (mesh == nullptr)
			{
				return;
			}
			mesh->Vertices = mesh->mVertices.empty() ? nullptr : mesh->mVertices.data();
			mesh->Indices = mesh->mIndices.empty() ? nullptr : mesh->mIndices.data();
		}

		void CopyStaticMesh(const StaticMesh& source, StaticMesh* target)
		{
			if (target == nullptr)
			{
				return;
			}
			*target = source;
			RefreshStaticMeshPointers(target);
		}

		bool LoadStaticMeshFile(const std::string& meshPath, StaticMesh* mesh, std::string* status)
		{
			if (mesh == nullptr)
			{
				return false;
			}

			mesh->Clear();
			mesh->Flags = 0;
			if (meshPath.empty())
			{
				if (status)
				{
					*status = "Missing mesh path";
				}
				return false;
			}

			bool loaded = false;
			if (HasPathExtension(meshPath, "obj"))
			{
				loaded = mesh->LoadObj(meshPath.c_str());
			}
			else if (HasPathExtension(meshPath, "flat"))
			{
				loaded = mesh->LoadFlat(meshPath.c_str());
			}
			else
			{
				if (status)
				{
					*status = "Unsupported mesh file";
				}
				return false;
			}

			if (!loaded || mesh->GetVertexCount() == 0 || mesh->GetTriangleCount() == 0)
			{
				if (status)
				{
					*status = "Failed to read mesh";
				}
				return false;
			}

			mesh->CalculateBoundingBox();
			mesh->CalculateWeightAverageNormals();
			RefreshStaticMeshPointers(mesh);
			return true;
		}
	}

	bool LoadMeshSimplificationSource(const std::string& meshPath, MeshSimplificationSource* source)
	{
		if (source == nullptr)
		{
			return false;
		}

		source->Mesh.Clear();
		source->Bounds = Box3::Empty();
		source->MaxSeparation = 1.0f;
		source->Status.clear();

		StaticMesh sourceMesh;
		if (!LoadStaticMeshFile(meshPath, &sourceMesh, &source->Status))
		{
			if (source->Status.empty())
			{
				source->Status = "Failed to read mesh";
			}
			return false;
		}

		source->Bounds = sourceMesh.BoundingVolume;
		source->MaxSeparation = std::max(BoundsMaxDim(source->Bounds), 1e-3f) * 1.25f;
		source->Mesh = std::move(sourceMesh);
		RefreshStaticMeshPointers(&source->Mesh);

		std::ostringstream status;
		status << "Ready: " << source->Mesh.GetTriangleCount() << " triangles";
		source->Status = status.str();
		return true;
	}

	bool BuildMeshSimplificationPanel(const MeshSimplificationParams& params, MeshSimplificationResult* result)
	{
		if (result == nullptr)
		{
			return false;
		}

		result->SourceMesh.Clear();
		result->SimplifiedMesh.Clear();
		result->SourceBounds = Box3::Empty();
		result->MaxSeparation = 1.0f;
		result->Status.clear();

		MeshSimplificationSource source;
		if (!LoadMeshSimplificationSource(params.MeshPath, &source))
		{
			result->Status = source.Status.empty() ? "Failed to read mesh" : source.Status;
			return false;
		}

		const float ratio = std::max(0.05f, std::min(params.Ratio, 1.0f));
		CopyStaticMesh(source.Mesh, &result->SourceMesh);
		CopyStaticMesh(source.Mesh, &result->SimplifiedMesh);

		if (ratio < 0.999f)
		{
			SimplificationConfig config;
			config.rate = ratio;
			if (!result->SimplifiedMesh.Simplify(config))
			{
				result->Status = "Simplification failed";
				return false;
			}
			result->SimplifiedMesh.CalculateWeightAverageNormals();
			RefreshStaticMeshPointers(&result->SimplifiedMesh);
		}

		result->SourceBounds = source.Bounds;
		result->MaxSeparation = source.MaxSeparation;

		std::ostringstream status;
		status << "Simplified: " << result->SourceMesh.GetTriangleCount()
			<< " -> " << result->SimplifiedMesh.GetTriangleCount()
			<< " triangles, ratio " << ratio;
		result->Status = status.str();
		return true;
	}
}
