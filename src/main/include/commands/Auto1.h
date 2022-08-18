#pragma once

#include <pathplanner/lib/PathPlanner.h>
#include <frc/trajectory/Trajectory.h>

class Auto1 {
  public:
    Auto1();
  
    frc::Trajectory GetTrajectory();

  private:
    frc::Trajectory m_Trajectory;
};