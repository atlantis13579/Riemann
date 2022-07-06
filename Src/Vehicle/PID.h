#pragma once

#include <algorithm>

// Proportional¨CIntegral¨CDerivative Controller
// https://en.wikipedia.org/wiki/PID_controller

struct PID_Calibration
{
	PID_Calibration()
	{
		kP = kI = kD = 1.0f;
		IntegralLower = -FLT_MAX;
		IntegralUpper = FLT_MAX;
		Lower = -FLT_MAX;
		Upper = FLT_MAX;
	}
	float kP;
	float kI;
	float kD;
	float IntegralLower;
	float IntegralUpper;
	float Lower;
	float Upper;
};


class PID_Controller
{
public:
	PID_Controller(const PID_Calibration &param)
	{
		SetCalibration(param);
		Reset();
	}

	~PID_Controller() {}

	void Reset()
	{
		mPrevError = 0.0f;
		mIntegral = 0.0f;
	}

	void SetCalibration(const PID_Calibration& param)
	{
		mParam = param;
	}

	float Compute(float dt, float Current, float Target)
	{
		float e = Target - Current;
		mIntegral = e * dt;
		mIntegral = std::max(std::min(mIntegral, mParam.IntegralUpper), mParam.IntegralLower);
		float output = mParam.kP * e + mParam.kI * mIntegral + mParam.kD * (e - mPrevError) / dt;
		mPrevError = e;
		return std::max(std::min(output, mParam.Upper), mParam.Lower);
	}

	void SetLimits(float lower, float upper)
	{
		mParam.Lower = lower;
		mParam.Upper = upper;
	}

private:
	PID_Calibration mParam;
	float			mPrevError;
	float			mIntegral;
};