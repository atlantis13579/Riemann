#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "../../Maths/Transform.h"
#include "../../Maths/Vector3.h"

namespace Riemann
{
	enum class UE5SimpleCollisionType
	{
		Sphere,
		Box,
		Capsule,
		Convex,
	};

	struct UE5TriangleMeshData
	{
		std::vector<Vector3> Vertices;
		std::vector<uint32_t> Indices;
	};

	struct UE5SimpleCollisionData
	{
		UE5SimpleCollisionType Type = UE5SimpleCollisionType::Sphere;
		Transform LocalTransform;
		Vector3 HalfExtent = Vector3::Zero();
		float Radius = 0.0f;
		float HalfHeight = 0.0f;
		UE5TriangleMeshData Convex;
	};

	struct UE5StaticMeshData
	{
		bool Success = false;
		std::string Error;
		std::vector<UE5TriangleMeshData> LODs;
		std::vector<UE5SimpleCollisionData> SimpleCollision;
		UE5TriangleMeshData ComplexCollision;
	};

	// UE uses Z-up coordinates while Riemann uses Y-up coordinates. Swapping
	// Y/Z also changes handedness, so rotations and triangle winding require
	// matching conversions rather than a position-only component swap.
	Vector3 UE5ToRiemannVector(const Vector3& Value);
	Quaternion UE5ToRiemannQuaternion(const Quaternion& Value);
	void ConvertUE5TriangleMeshToRiemann(UE5TriangleMeshData* Mesh);

	// Loads an uncooked UE5.4 StaticMesh package in Riemann Y-up coordinates.
	// Distance units remain UE centimeters.
	UE5StaticMeshData LoadUE5StaticMesh(const char* Filename);
}
