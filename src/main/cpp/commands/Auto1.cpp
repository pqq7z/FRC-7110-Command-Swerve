#include "commands/Auto1.h"
#include "Constants.h"

using namespace pathplanner;

frc::Trajectory Auto1::GetTrajectory() {
  PathPlannerTrajectory path = PathPlanner::loadPath("TestPath", AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
  m_Trajectory = path.asWPILibTrajectory();
  return m_Trajectory;
}