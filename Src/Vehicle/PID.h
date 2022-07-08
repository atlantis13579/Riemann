#pragma once

#include <float.h>
#include <algorithm>

// Proportional¨CIntegral¨CDerivative Controller
// https://en.wikipedia.org/wiki/PID_controller

struct PID_Calibration
{
	PID_Calibration()
	{
		kP = kI = kD = 1.0f;
		LowIntegral = -FLT_MAX;
		HighIntegral = FLT_MAX;
		LowValue = -FLT_MAX;
		HighValue = FLT_MAX;
	}
	float kP;
	float kI;
	float kD;
	float LowIntegral;
	float HighIntegral;
	float LowValue;
	float HighValue;
};


class PID_Controller
{
public:
	PID_Controller(const PID_Calibration& param)
	{
		SetCalibration(param);
		Reset();
	}

	PID_Controller(float _kP, float _kI, float _kD, float lowIntegral, float highIntegral, float low, float high)
	{
		PID_Calibration param;
		param.kP = _kP;
		param.kI = _kI;
		param.kD = _kD;
		param.LowIntegral = low;
		param.HighIntegral = high;
		param.LowValue = low;
		param.HighValue = high;
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
		mIntegral += e * dt;
		mIntegral = std::max(std::min(mIntegral, mParam.HighIntegral), mParam.LowIntegral);
		float output = mParam.kP * e + mParam.kI * mIntegral + mParam.kD * (e - mPrevError) / dt;
		mPrevError = e;
		return std::max(std::min(output, mParam.HighValue), mParam.LowValue);
	}

	void SetLimits(float lower, float high)
	{
		mParam.LowValue = lower;
		mParam.HighValue = high;
	}

private:
	PID_Calibration mParam;
	float			mPrevError;
	float			mIntegral;
};
