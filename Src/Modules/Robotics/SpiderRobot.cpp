#include "SpiderRobot.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace Riemann
{
	void ApplySpiderAction(SpiderRobotControl& Control, const std::vector<float>& Action)
	{
		if (Action.size() < SpiderMotorCount)
			throw std::invalid_argument("Spider action has too few motors");
		for (size_t M = 0; M < SpiderMotorCount; ++M)
			Control.TargetOffset[M] = std::max(-0.7, std::min(0.7,
				0.5 * Control.TargetOffset[M] + 0.5 * double(Action[M])));
	}

	ArticulatedRobotSettings CreateSpiderRobotSettings(const std::shared_ptr<SpiderRobotControl>& Control)
	{
		if (!Control)
			throw std::invalid_argument("Spider control is required");
		ArticulatedRobotSettings Settings;
		Settings.TimeStep = 0.001;
		Settings.Friction = 1.2;
		Settings.Links.reserve(1 + SpiderMotorCount);
		Settings.Pose.resize(1 + SpiderMotorCount);
		Settings.Feet.reserve(SpiderLegCount);

		RagdollLinkSettings Body;
		Body.Joint = RagdollReducedJointType::Floating;
		Body.Mass = 6.0f;
		Body.Inertia = Matrix3(0.22f, 0.22f, 0.36f);
		Body.ParentToJoint.quat.FromRotateX(-1.5707963267948966f);
		Body.ParentToJoint.pos = Body.ParentToJoint.quat * Vector3(0, 0, 0.50f);
		Settings.Links.push_back(Body);

		for (size_t Leg = 0; Leg < SpiderLegCount; ++Leg)
		{
			const float Angle = float(Leg * 6.283185307179586 / SpiderLegCount);
			const int Coxa = int(1 + 3 * Leg);
			RagdollLinkSettings Yaw;
			Yaw.Parent = 0;
			Yaw.Joint = RagdollReducedJointType::Revolute;
			Yaw.ParentToJoint.pos = Vector3(0.28f * std::cos(Angle), 0.28f * std::sin(Angle), 0);
			Yaw.ParentToJoint.quat.FromRotateZ(Angle);
			Yaw.Axis = Vector3::UnitZ();
			Yaw.CenterOfMass = Vector3(0.09f, 0, 0);
			Yaw.Mass = 0.35f;
			Yaw.Inertia = Matrix3(0.0015f, 0.003f, 0.003f);
			Yaw.Damping = 0.15;
			Settings.Links.push_back(Yaw);

			RagdollLinkSettings Hip;
			Hip.Parent = Coxa;
			Hip.Joint = RagdollReducedJointType::Revolute;
			Hip.ParentToJoint.pos = Vector3(0.18f, 0, 0);
			Hip.Axis = Vector3::UnitY();
			Hip.CenterOfMass = Vector3(0.19f, 0, 0);
			Hip.Mass = 0.8f;
			Hip.Inertia = Matrix3(0.003f, 0.02f, 0.02f);
			Hip.Damping = 0.2;
			Settings.Links.push_back(Hip);

			RagdollLinkSettings Knee;
			Knee.Parent = Coxa + 1;
			Knee.Joint = RagdollReducedJointType::Revolute;
			Knee.ParentToJoint.pos = Vector3(0.38f, 0, 0);
			Knee.Axis = Vector3::UnitY();
			Knee.CenterOfMass = Vector3(0.375f, 0, 0);
			Knee.Mass = 0.6f;
			Knee.Inertia = Matrix3(0.004f, 0.04f, 0.04f);
			Knee.Damping = 0.2;
			Settings.Links.push_back(Knee);

			const size_t M = 3 * Leg;
			Control->Home[M] = 0;
			Control->Home[M + 1] = -0.6;
			Control->Home[M + 2] = 1.75;
			Control->Lower[M] = -0.45;
			Control->Lower[M + 1] = -1.0;
			Control->Lower[M + 2] = 0.5;
			Control->Upper[M] = 0.45;
			Control->Upper[M + 1] = 0.5;
			Control->Upper[M + 2] = 2.3;
			Control->TorqueLimit[M] = 30;
			Control->TorqueLimit[M + 1] = 60;
			Control->TorqueLimit[M + 2] = 60;
			for (size_t J = 0; J < 3; ++J)
				Settings.Pose[Coxa + J].Position = Control->Home[M + J];
			Settings.Feet.push_back({size_t(Coxa + 2), Vector3(0.75f, 0, 0), 0.035f});
		}

		Settings.Actuate = [Control](const FeatherstoneSolver& Tree, const Vector3&, size_t Tick,
			std::vector<double>& Torque)
		{
			Control->Phase = std::fmod(float(Tick) * 0.001f, 1.0f);
			for (size_t M = 0; M < SpiderMotorCount; ++M)
			{
				const size_t Link = M + 1;
				const int Dof = Tree.GetLinkDofOffset(Link);
				const double Q = Tree.GetConfiguration()[Link].Position;
				const double V = Tree.GetVelocity()[Dof];
				const double Target = std::max(Control->Lower[M], std::min(Control->Upper[M],
					Control->Home[M] + Control->TargetOffset[M]));
				const double LimitError = std::max(Control->Lower[M], std::min(Control->Upper[M], Q)) - Q;
				const double Effort = 120 * (Target - Q) - 5 * V + 800 * LimitError;
				Torque[Dof] = std::max(-Control->TorqueLimit[M], std::min(Control->TorqueLimit[M], Effort));
			}
		};
		return Settings;
	}

	std::vector<float> ObserveSpiderRobot(const FeatherstoneSolver& Tree, const Vector3& Command,
		const SpiderRobotControl& Control)
	{
		if (Tree.GetLinkCount() != 1 + SpiderMotorCount)
			throw std::invalid_argument("Unexpected spider joint count");
		std::vector<float> Observation(14 + 3 * SpiderMotorCount);
		const Transform Base = Tree.GetLinkTransform(0);
		const Quaternion Inverse = Base.quat.Conjugate();
		const std::vector<double>& Velocity = Tree.GetVelocity();
		const Vector3 Angular(static_cast<float>(Velocity[0]), static_cast<float>(Velocity[1]), static_cast<float>(Velocity[2]));
		const Vector3 Linear(static_cast<float>(Velocity[3]), static_cast<float>(Velocity[4]), static_cast<float>(Velocity[5]));
		const Vector3 LocalLinear = Inverse * Linear;
		const Vector3 LocalAngular = Inverse * Angular;
		const Vector3 Gravity = Inverse * -Vector3::UnitY();
		const Vector3 LocalCommand = Inverse * Command;
		for (int I = 0; I < 3; ++I)
		{
			Observation[I] = LocalLinear[I];
			Observation[3 + I] = LocalAngular[I];
			Observation[6 + I] = Gravity[I];
			Observation[9 + I] = LocalCommand[I];
		}
		for (size_t M = 0; M < SpiderMotorCount; ++M)
		{
			const size_t Link = M + 1;
			Observation[12 + M] = float(Tree.GetConfiguration()[Link].Position - Control.Home[M]);
			Observation[12 + SpiderMotorCount + M] = float(Velocity[Tree.GetLinkDofOffset(Link)]);
			Observation[12 + 2 * SpiderMotorCount + M] = float(Control.TargetOffset[M]);
		}
		Observation[84] = std::sin(6.283185307179586f * Control.Phase);
		Observation[85] = std::cos(6.283185307179586f * Control.Phase);
		return Observation;
	}
}
