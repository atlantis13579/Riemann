#pragma once

#include "PBDSolver.h"

namespace Riemann
{
	struct XPBDSolverSettings : PBDSolverSettings
	{
		// Compliance is inverse contact stiffness, in meters per Newton.
		float ContactCompliance = 0.0f;
	};

	class XPBDSolver : public PBDSolver
	{
	public:
		explicit XPBDSolver(const XPBDSolverSettings& Settings = XPBDSolverSettings());
		void SetXPBDSettings(const XPBDSolverSettings& Settings);
		XPBDSolverSettings GetXPBDSettings() const
		{
			XPBDSolverSettings Settings = m_XPBDSettings;
			static_cast<PBDSolverSettings&>(Settings) = PBDSolver::GetSettings();
			return Settings;
		}
		virtual bool CanSleep(const RigidBodyDynamic* Body) const override;

	protected:
		float GetContactCompliance() const override { return m_XPBDSettings.ContactCompliance; }

	private:
		XPBDSolverSettings m_XPBDSettings;
	};
}