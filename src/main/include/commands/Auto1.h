#pragma once

#include <pathplanner/lib/PathPlanner.h>
#include <frc/trajectory/Trajectory.h>

class Auto1 {
  public:
    Auto1();

    /**
     * @brief Gets the trajectory of Pathplanner Path as frc::Trajectory
     * 
     * @return frc::Trajectory (the path)
     */
    frc::Trajectory GetTrajectory();

  private:
    frc::Trajectory m_Trajectory;
};