#include "SnowSurfaceBuilder.h"
#include "../Src/Fluid/MarchingCubes.h"
#include <algorithm>
#include <cmath>

namespace Riemann
{
	bool BuildSnowSurface(const SnowSimulation& Simulation, SnowQuality Quality, RenderMeshDesc& Mesh)
	{
		Mesh = {}; Mesh.Id = "snow.surface"; Mesh.WorldTransform = Transform::Identity();
		Mesh.Color = Vector4(1,1,1,1); Mesh.SnowFraction = Simulation.GetSettings().Material.SnowFraction;
		Mesh.FillMode = RenderFillMode::Solid;
		const auto& Particles = Simulation.GetParticles(); if (Particles.empty()) return true;
		const float Dx = Simulation.GetSettings().CellSize;
		const bool Sand = Mesh.SnowFraction < 0.5f;
		const float Radius = Dx * (Sand ? 1.3f : (Quality == SnowQuality::Realtime ? 1.1f : 0.95f));
		const float Spacing = Dx * (Quality == SnowQuality::Realtime ? 0.55f : 0.45f);
		Vector3 Minimum(1.e10f), Maximum(-1.e10f);
		for (const auto& P : Particles)
			for (int A=0;A<3;++A) { Minimum[A]=std::min(Minimum[A],P.Position[A]); Maximum[A]=std::max(Maximum[A],P.Position[A]); }
		Minimum -= Vector3(Radius+Spacing); Maximum += Vector3(Radius+Spacing);
		for (int A=0;A<3;++A) Minimum[A]=floorf(Minimum[A]/Spacing)*Spacing;
		const Vector3 Span=Maximum-Minimum;
		Vector3i Size((int)ceilf(Span.x/Spacing)+1,(int)ceilf(Span.y/Spacing)+1,(int)ceilf(Span.z/Spacing)+1);
		const size_t Count=(size_t)Size.x*Size.y*Size.z;
		if (Count>4000000) return false;
		std::vector<float> Density(Count,0);
		const float Normalization=315/(64*3.14159265359f*Radius*Radius*Radius),InverseRadius=1/(Radius*Radius);
		for (const auto& P : Particles)
		{
			const Vector3 Position(P.Position.x,P.Position.y,P.Position.z);
			const Vector3 Grid=(Position-Minimum)/Spacing;
			const float GridRadius=Radius/Spacing;
			const int X0=std::max(0,(int)ceilf(Grid.x-GridRadius)),X1=std::min(Size.x-1,(int)floorf(Grid.x+GridRadius));
			const int Y0=std::max(0,(int)ceilf(Grid.y-GridRadius)),Y1=std::min(Size.y-1,(int)floorf(Grid.y+GridRadius));
			const int Z0=std::max(0,(int)ceilf(Grid.z-GridRadius)),Z1=std::min(Size.z-1,(int)floorf(Grid.z+GridRadius));
			const float Scale=P.Volume*Normalization;
			for (int Z=Z0;Z<=Z1;++Z)
				for (int Y=Y0;Y<=Y1;++Y)
					for (int X=X0;X<=X1;++X)
					{
						const Vector3 Delta=Minimum+Vector3((float)X,(float)Y,(float)Z)*Spacing-Position;
						const float Q=1-Delta.SquareLength()*InverseRadius;
						if(Q>0) Density[((size_t)Z*Size.y+Y)*Size.x+X]+=Scale*Q*Q*Q;
					}
		}
		MarchingCubesInput Input;Input.Origin=Minimum;Input.Spacing=Vector3(Spacing);Input.Resolution=Size;
		Input.Values=Density.data();Input.ValueCount=Density.size();Input.IsoValue=0.45f;Input.HighValuesInside=true;
		FluidSurfaceMesh Surface;if(!MarchingCubes::Extract(Input,&Surface)) return false;
		Mesh.Vertices.reserve(Surface.Positions.size());
		const auto& Settings=Simulation.GetSettings();
		for(size_t I=0;I<Surface.Positions.size();++I)
		{
			Vector3 Position=Surface.Positions[I];
			// Density kernels extend beyond particles; the closed tray also clips their visual support.
			for(int A=0;A<3;++A) Position[A]=std::max(Settings.BoundsMin[A]+1.5f*Dx,std::min(Settings.BoundsMax[A]-1.5f*Dx,Position[A]));
			Mesh.Vertices.emplace_back(Position,Surface.Normals[I]);
		}
		Mesh.Indices=std::move(Surface.Indices);return true;
	}
	bool BuildSandSurface(const SandSimulation& Simulation,RenderMeshDesc& Mesh)
	{
		Mesh={};Mesh.Id="snow.surface";Mesh.WorldTransform=Transform::Identity();
		Mesh.Color=Vector4(1,1,1,1);Mesh.SnowFraction=0;Mesh.FillMode=RenderFillMode::Solid;
		const int N=Simulation.GetResolution(),Stride=N+1;const float Dx=Simulation.GetCellSize();
		const auto& Heights=Simulation.GetHeights();if(Heights.empty()) return true;
		const auto Height=[&](int X,int Z) {
			const int X0=std::max(0,X-1),X1=std::min(N-1,X),Z0=std::max(0,Z-1),Z1=std::min(N-1,Z);
			return (Heights[Z0*N+X0]+Heights[Z0*N+X1]+Heights[Z1*N+X0]+Heights[Z1*N+X1])*0.25f;
		};
		Mesh.Vertices.reserve(Stride*Stride);Mesh.Indices.reserve(N*N*6);
		for(int Z=0;Z<=N;++Z) for(int X=0;X<=N;++X)
		{
			const int X0=std::max(0,X-1),X1=std::min(N,X+1),Z0=std::max(0,Z-1),Z1=std::min(N,Z+1);
			Vector3 Normal(-(Height(X1,Z)-Height(X0,Z))/((X1-X0)*Dx),1,-(Height(X,Z1)-Height(X,Z0))/((Z1-Z0)*Dx));
			Normal.Normalize();Mesh.Vertices.emplace_back(Vector3(-4+X*Dx,Height(X,Z),-4+Z*Dx),Normal);
			if(X<N && Z<N)
			{
				const uint32_t A=Z*Stride+X,B=A+1,C=A+Stride,D=C+1;
				Mesh.Indices.insert(Mesh.Indices.end(),{A,C,B,B,C,D});
			}
		}
		return true;
	}
	RenderMeshDesc BuildSnowCapsule(const SnowCapsule& Capsule)
	{
		RenderMeshDesc Mesh;Mesh.Id="snow.capsule";Mesh.Color=Vector4(0.93f,0.34f,0.075f,1);
		Mesh.WorldTransform=Transform(Vector3(Capsule.Center.x,Capsule.Center.y,Capsule.Center.z));Mesh.FillMode=RenderFillMode::Solid;
		const int Rings=24,Sides=40;const float Pi=3.14159265359f;
		for(int Y=0;Y<=Rings;++Y)
		{
			const float Angle=Pi*Y/Rings-Pi*0.5f,Height=sinf(Angle),Width=cosf(Angle);
			for(int X=0;X<=Sides;++X)
			{
				const float A=2*Pi*X/Sides;const Vector3 Normal(Width*cosf(A),Height,Width*sinf(A));
				Vector3 Position=Normal*Capsule.Radius;Position.y+=(Y<Rings/2 ? -Capsule.HalfLength : Capsule.HalfLength);
				Mesh.Vertices.emplace_back(Position,Normal);
			}
		}
		for(int Y=0;Y<Rings;++Y) for(int X=0;X<Sides;++X)
		{
			const uint32_t A=Y*(Sides+1)+X,B=A+1,C=A+Sides+1,D=C+1;
			Mesh.Indices.insert(Mesh.Indices.end(),{A,C,B,B,C,D});
		}
		return Mesh;
	}
}
