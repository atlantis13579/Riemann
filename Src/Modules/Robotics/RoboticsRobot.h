#pragma once

#include "ArticulatedRobotController.h"
#include <array>
#include <memory>

namespace Riemann
{
	constexpr size_t RoboticsLegCount = 8;
	constexpr size_t RoboticsMotorCount = RoboticsLegCount * 3;

	// Motors are ordered by leg, then yaw, hip and knee.
	struct RoboticsRobotControl
	{
		std::array<double, RoboticsMotorCount> TargetOffset{};
		float Phase = 0;
		std::array<double, RoboticsMotorCount> Home{};
		std::array<double, RoboticsMotorCount> Lower{};
		std::array<double, RoboticsMotorCount> Upper{};
		std::array<double, RoboticsMotorCount> TorqueLimit{};
	};

	ArticulatedRobotSettings CreateRoboticsRobotSettings(const std::shared_ptr<RoboticsRobotControl>& Control);
	void ApplyRoboticsAction(RoboticsRobotControl& Control, const std::vector<float>& Action);
	std::vector<float> ObserveRoboticsRobot(const FeatherstoneSolver& Tree, const Vector3& Command,
		const RoboticsRobotControl& Control);
}
