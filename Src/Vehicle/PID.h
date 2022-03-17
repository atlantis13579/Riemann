#pragma once

struct PID_Calibration
{
	float kP;
	float kI;
	float kD;
};

struct PID_State
{
	float LastError;
	float Integral;
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
	}

	float Process(float dt, float Error)
	{
		mIntegral += Error * dt;
		float output = mParam.kP * Error + mParam.kI * mIntegral + mParam.kD * (Error - mPrevError) / dt;
		mPrevError = Error;
		return output;
	}

private:
	PID_Calibration mParam;
	float			mPrevError;
	float			mIntegral;
};