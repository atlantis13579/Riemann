#pragma once

#include <cstdint>
#include <string>
#include "../Src/Destruction/DestructionSet.h"

namespace Riemann
{
	class SceneWorld;
	// UI order is SI, LCP, PBD, XPBD; backend order is CPU, SIMD, ComputeShader, CUDA.
	// These indices are deliberately independent of the engine enum values.
	struct SolverPanelState
	{
		int ResolutionIndex = 0;
		int BackendIndex = 0;
		int ActiveResolutionIndex = 0;
		int ActiveBackendIndex = 0;
		int ActiveRequestedBackendIndex = 0;
		bool AllowFallback = true;
		bool PersistentContacts = false;
		bool TriangleContacts = true;
		bool CollisionFiltering = false;
		bool ShapeOffsets = true;
		bool SpeculativeContacts = false;
		bool DistanceFieldDemo = false;
		bool StrandDemo = false, HasStrands = false;
		bool StrandCoupled = true;
		bool StrandContacts=false, StrandSelfContacts=false, StrandContinuous=true, StrandFriction=true;
		int StrandLod = 0;
		float StrandRootTwist = 0;
		uint64_t StrandGuides = 0, StrandConstraintSolves = 0;
		uint64_t StrandActiveGuides = 0, StrandRenderCurves = 0;
		uint64_t StrandContactRows=0, StrandContactCandidates=0, StrandContactRefinements=0;
		double StrandContactPenetration=0;
		double StrandEnergy = 0, StrandStretchShear = 0, StrandBendTwist = 0;
		bool PhysicsFieldsDemo = false, PhysicsFieldFreeze = false, FieldStrainPulse = false, FieldVelocityPulse = false;
		uint64_t FieldSamples = 0, FieldBodyApplications = 0, FieldBrokenBonds = 0;
		bool AuthoredDestructionDemo = false;
		bool DestructionEventsDemo = false;
		bool DestructionCacheDemo = false, DestructionRecording = false, DestructionPlaying = false;
		DestructionPlaybackCommand DestructionCacheCommand = DestructionPlaybackCommand::None;
		float DestructionSeekFraction = 0;
		double DestructionCacheTime = 0, DestructionCacheDuration = 0;
		uint64_t DestructionEvents = 0, DestructionDroppedEvents = 0;
		bool CharacterDemo = false, CharacterBox = false;
		bool VehicleDynamicsDemo = false;
		bool ReducedRagdollDemo = false;
		bool HasReducedRagdolls = false;
		bool PhysicalAnimationDemo = false, HasPhysicalAnimation = false;
		bool AnimationPassive = false;
		float AnimationArmStrength = 1, AnimationPhysicsBlend = 0.5f;
		int AnimationState = 0;
		bool ClothModelDemo = false, HasClothDemo = false;
		int ClothAlgorithm = 0;
		float ClothIterations = 12, ClothSubsteps = 1, ClothDamping = 0.995f;
		float ClothStretchUCompliance = 0.0001f, ClothStretchVCompliance = 0.0002f;
		float ClothShearCompliance = 0.01f, ClothBendingCompliance = 25;
		bool ClothWind = true, ClothAnimation = false, ClothTethers = true, ClothSoftMaterial = false;
		bool ClothSurfaceContacts = false, ClothFriction = true;
		bool ClothSelfCollision = false, ClothInterCollision = false;
		bool ClothContinuous = false;
		bool ClothRigidCoupling = false;
		bool SoftBodyAttachments = false, CompliantAttachments = false;
		bool RigidAttachments = false;
		bool SoftBodyContacts = false;
		bool SoftRigidContacts = false;
		bool CalibratedSoftMaterial = false, StVKSoftMaterial = false;
		bool PreparedSoftAsset = false, EmbeddedSoftRender = true;
		bool CorotatedSoftMaterial = false, ProjectiveSoftMaterial = false;
		bool SequentialWarmStart = true;
		bool UnifiedConstraints = true;
		bool FrictionPatches = false;
		bool JointDrives = true;
		bool JointBreaking = true;
		bool CCDContinuation = false;
		bool TemporalSubsteps = false;
		bool SupportStabilization = false;
		bool IslandSleeping = false;
		bool Available[4][4] = {
			{ true, false, false, false },
			{ true, false, false, false },
			{ true, false, false, false },
			{ true, false, false, false },
		};
		bool Enabled = false;
		bool SolverSwitchable = false;
		bool SolverChanged = false;
		bool Pending = false;
		std::string Status;
		uint32_t SceneGeneration = 0;
	};

	// Draw inside an existing imgui scroll area. A changed solver selection or the
	// cloth apply button requests work on the simulation thread before its next step.
	bool DrawSolverPanel(SolverPanelState& State);
	void RefreshSolverPanelTelemetry(SolverPanelState& State, const SceneWorld& Scene);
}
