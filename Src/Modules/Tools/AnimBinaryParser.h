#pragma once

#include <string>
#include <vector>

#include "../../Maths/Vector3.h"
#include "../../Maths/Quaternion.h"

typedef std::vector<std::pair<int, Vector3>>	ScaleKeyframes;
typedef std::vector<std::pair<int, Vector3>>	PositionKeyframes;
typedef std::vector<std::pair<int, Quaternion>> RotationKeyframes;
typedef std::vector<std::pair<int, float>>		YawKeyframes;

class  BoneChannel
{
public:
	std::string				boneName;
	ScaleKeyframes			scaleKeys;
	PositionKeyframes		positionKeys;
	RotationKeyframes		rotationKeys;
	int						parentPos;
};

class  MotionChannel
{
public:
	PositionKeyframes		positionKeys;
	//RotationKeyframes		rotationKeys;
	YawKeyframes			yawKeys;
};

class AnimTreeData
{
public:
	virtual ~AnimTreeData() {}

	std::vector<BoneChannel>	channels;
	MotionChannel				motion;

public:
	static AnimTreeData* Deserialize(const char* filepath);
};
