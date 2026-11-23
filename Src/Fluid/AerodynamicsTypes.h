#pragma once

#include <cstdint>
#include <vector>

#include "FluidBackend.h"
#include "../Maths/Quaternion.h"
#include "../Maths/Index3.h"

namespace Riemann
{
	enum class AerodynamicsAlgorithm : uint8_t { Barotropic, Incompressible };

	struct AerodynamicsEmitter
	{
		// Density is transported tracer concentration, independent of air mass density.
		Vector3 Position = Vector3(0, 0.55f, 0);
		Vector3 Velocity = Vector3(0, 1.8f, 0);
		float Radius = 0.32f;
		float Density = 1.4f;
		float Temperature = 1.0f;
	};

	struct AerodynamicsObstacle
	{
		enum class Shape { Sphere, Box };
		Shape Type = Shape::Sphere;
		Vector3 Center = Vector3::Zero();
		float Radius = 0.5f;
		Vector3 HalfExtent = Vector3(0.5f);
		Quaternion Rotation = Quaternion::One();
		Vector3 LinearVelocity = Vector3::Zero();
		Vector3 AngularVelocity = Vector3::Zero();
	};

	struct AerodynamicsRigidLoad
	{
		Vector3 Force = Vector3::Zero();
		Vector3 Torque = Vector3::Zero();
	};

	// A deforming, two-sided solid surface. Thickness is a numerical half-width.
	struct AerodynamicsSurfaceMesh
	{
		std::vector<Vector3> Positions, Velocities;
		// Empty denotes prescribed motion; pinned vertices have zero inverse mass.
		std::vector<float> InverseMasses;
		std::vector<Index3> Triangles;
		float Thickness = 0;
	};

	struct AerodynamicsSimulationSettings
	{
		explicit AerodynamicsSimulationSettings(AerodynamicsAlgorithm Algorithm = AerodynamicsAlgorithm::Barotropic);

		AerodynamicsAlgorithm Algorithm;

		FluidSimulationMethod Method;
		FluidBackend Backend;
		bool AllowBackendFallback;
		Vector3 BoundsMin;
		Vector3 BoundsMax;
		Vector3i Resolution;
		Vector3 Gravity;
		float AmbientDensity;
		float AmbientPressure;
		float SpeedOfSound;
		float AdiabaticExponent;
		float DynamicViscosity;
		float ArtificialViscosity;
		float ParticleSpacing;
		float SmoothingRadius;
		float BoundaryRestitution;
		float MaxTimeStep;
		float AcousticCFL;
		int MaxSubsteps;
		// Incompressible transport parameters; Density is tracer concentration.
		int PressureIterations = 40;
		float Buoyancy = 3.0f, TracerWeight = 0.15f, Vorticity = 2.5f;
		float DensityDissipation = 0.10f, Cooling = 0.45f;
		std::vector<AerodynamicsEmitter> Emitters;
		std::vector<AerodynamicsObstacle> Obstacles;
	};

	struct AirParticle
	{
		AirParticle();

		Vector3 Position;
		Vector3 ReferencePosition;
		Vector3 Velocity;
		Vector3 Acceleration;
		float Mass;
		float ReferenceMass;
		float DensityNormalization;
		float Density;
		float Pressure;
	};

	struct AerodynamicSample
	{
		AerodynamicSample();

		Vector3 Velocity;
		Vector3 PressureGradient;
		float TracerDensity = 0, Temperature = 0;
		float Density;
		float Pressure;
	};

	struct AerodynamicSurfaceSample
	{
		AerodynamicSurfaceSample();

		Vector3 Position;
		Vector3 Normal;
		Vector3 Velocity;
		float Area;
	};

	struct AerodynamicForceResult
	{
		AerodynamicForceResult();

		Vector3 Force;
		Vector3 Torque;
	};

}
