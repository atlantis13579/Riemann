#pragma once

#include <algorithm>

struct PID_Calibration
{
	float kP;
	float kI;
	float kD;
};

class PID_Controller
{
public:
	PID_Controller(float _kP, float _kI, float _kD)
	{
		mParam.kP = _kP;
		mParam.kI = _kI;
		mParam.kD = _kD;
		Reset();
	}

	~PID_Controller() {}

	void Reset()
	{
		mPrevError = 0.0f;
		mIntegral = 0.0f;
		mLower = -FLT_MAX;
		mUpper = FLT_MAX;
	}

	float Compute(float dt, float Current, float Target)
	{
		float e = Target - Current;
		mIntegral += e * dt;
		float output = mParam.kP * e + mParam.kI * mIntegral + mParam.kD * (e - mPrevError) / dt;
		mPrevError = e;
		return std::max(std::min(output, mUpper), mLower);
	}

	void SetLimits(float lower, float upper)
	{
		mLower = lower;
		mUpper = upper;
	}

private:
	PID_Calibration mParam;
	float			mPrevError;
	float			mIntegral;
	float			mLower, mUpper;
};