
#pragma once

#include <stdint.h>
#include <vector>
#include "PrimitiveType.h"
#include "Plane3.h"
#include "../Maths/Vector3.h"
#include "../Maths/Box3.h"
#include "../Maths/Matrix3.h"

// #define USE_EDGE_DATA

namespace Riemann
{
	struct ConvexMeshFace
	{
		explicit ConvexMeshFace(const Plane3& pl, uint8_t nVerties, uint16_t first_idx)
		{
			plane = pl;
			numVerties = nVerties;
			first = first_idx;
		}
		Plane3			plane;		// The Noraml always point out of the Convex hull
		uint16_t		first;		// offset in Indices[] point to the first element
		uint8_t			numVerties;	// face vertices : Vertices[Indices[first]], Vertices[Indices[first + 1]], ..., Vertices[Indices[first + numVerties - 1]]
		uint8_t			padding;	// unused, to keep 20 bytes size
	};

	struct ConvexMeshEdge
	{
		explicit ConvexMeshEdge(uint16_t _s, uint16_t _e)
		{
			s = _s;
			e = _e;
		}
		uint16_t		s;
		uint16_t		e;
	};

	static_assert(sizeof(ConvexMeshEdge) == 4, "sizeof(HullEdge3d) not right");
	static_assert(sizeof(ConvexMeshFace) == 20, "sizeof(HullFace3d) not right");

	class ConvexMesh
	{
	public:
		Vector3*			Vertices;
		ConvexMeshEdge*		Edges;
		ConvexMeshFace*		Faces;
		uint8_t*			Indices;
		uint16_t			NumVertices;
		uint16_t			NumEdges;
		uint16_t			NumFaces;
		uint16_t			NumIndices;
		std::vector<char>	Buffers;
		Box3				Bounds;

		ConvexMesh()
		{
			Release();
		}

		static constexpr PrimitiveType	StaticType()
		{
			return PrimitiveType::CONVEX_MESH;
		}

		void		Release()
		{
			NumVertices = NumFaces = NumEdges = NumIndices = 0;
			Buffers.clear();
		}

		int			EulerNumber() const
		{
			return NumVertices - NumEdges + NumFaces;
		}

		void 		SetConvexData(Vector3* verts, uint16_t nVerties,
			ConvexMeshFace* faces, uint16_t nFaces,
			uint16_t* edges, uint16_t nEdges,
			uint8_t* indices, uint16_t nIndices,
			bool shared_mem);

		uint16_t	GetNumVertices() const
		{
			return NumVertices;
		}

		uint16_t	GetNumEdges() const
		{
			return NumEdges;
		}

		uint16_t	GetNumFaces() const
		{
			return NumFaces;
		}

		uint16_t	GetNumIndices() const
		{
			return NumIndices;
		}

		bool		ValidateStructure() const;

		Box3		ComputeBoundingVolume() const;

		bool		IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const;
		bool		IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const;
		bool		IntersectSphere(const Vector3& iCenter, float iRadius) const;
		bool		IntersectCapsule(const Vector3& X0, const Vector3& X1, float rRadius) const;
		bool		IntersectConvex(const ConvexMesh* convex) const;

		bool		CalculateVolumeProperties(MassParameters* p, float Density) const;

		Vector3		GetCenter() const
		{
			return Bounds.GetCenter();
		}

		Vector3		GetSupport(const Vector3& Direction) const;
		int			GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const;

		void		GetMesh(std::vector<Vector3>& iVertices, std::vector<uint16_t>& iIndices, std::vector<Vector3>& _Normals);
		void		GetWireframe(std::vector<Vector3>& iVertices, std::vector<uint16_t>& iIndices);
	};
}