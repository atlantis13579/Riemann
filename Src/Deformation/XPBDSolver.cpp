
#include "XPBDSolver.h"

#include <cmath>

namespace Riemann
{
	XPBDSolver::XPBDSolver(const XPBDSolverSettings& Settings)
		: PBDSolver(Settings)
	{
		SetXPBDSettings(Settings);
	}

	void XPBDSolver::SetXPBDSettings(const XPBDSolverSettings& Settings)
	{
		PBDSolver::SetSettings(Settings);
		static_cast<PBDSolverSettings&>(m_XPBDSettings) = PBDSolver::GetSettings();
		m_XPBDSettings.ContactCompliance = std::isfinite(Settings.ContactCompliance) && Settings.ContactCompliance >= 0.0f
			? Settings.ContactCompliance : 0.0f;
		ResetState();
	}

	bool XPBDSolver::CanSleep(const RigidBodyDynamic* Body) const
	{
		// Contact velocity stabilization may reach zero before the compliant
		// position has equilibrated under load. Keep soft contacts integrating.
		return m_XPBDSettings.ContactCompliance == 0.0f && PBDSolver::CanSleep(Body);
	}
}
