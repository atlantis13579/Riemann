#pragma once

#include "ArticulatedRobotController.h"
#include <array>
#include <memory>

namespace Riemann
{
	constexpr size_t SpiderLegCount = 8;
	constexpr size_t SpiderMotorCount = SpiderLegCount * 3;

	// Motors are ordered by leg, then yaw, hip and knee.
	struct SpiderRobotControl
	{
		std::array<double, SpiderMotorCount> TargetOffset{};
		float Phase = 0;
		std::array<double, SpiderMotorCount> Home{};
		std::array<double, SpiderMotorCount> Lower{};
		std::array<double, SpiderMotorCount> Upper{};
		std::array<double, SpiderMotorCount> TorqueLimit{};
	};

	ArticulatedRobotSettings CreateSpiderRobotSettings(const std::shared_ptr<SpiderRobotControl>& Control);
	void ApplySpiderAction(SpiderRobotControl& Control, const std::vector<float>& Action);
	std::vector<float> ObserveSpiderRobot(const FeatherstoneSolver& Tree, const Vector3& Command,
		const SpiderRobotControl& Control);
}
