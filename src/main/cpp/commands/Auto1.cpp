#include "commands/Auto1.h"
#include "Constants.h"

using namespace pathplanner;

Auto1::Auto1(){  
  PathPlannerTrajectory path = PathPlanner::loadPath("TestPath", AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
  m_Trajectory = path.asWPILibTrajectory();
  }

frc::Trajectory Auto1::GetTrajectory() {
  return m_Trajectory;
}