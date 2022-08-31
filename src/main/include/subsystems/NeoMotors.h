#pragma once

#include <rev/CANSparkMax.h>
#include <wpi/numbers>
#include <frc/controller/BangBangController.h>
#include <frc/controller/PIDController.h>

namespace FRC7110 {
  class NeoMotor : public rev::CANSparkMax, public rev::SparkMaxRelativeEncoder{
    public:
    
      explicit NeoMotor(int Id, rev::CANSparkMax::MotorType type);

      /**
       * @brief this method is used to set the conversion from RPM to MPS
       * to do this we get the gear ratio * 2pi * wheel diameter(meters)
       * 
       * @param ratio
       */
      void SetRPM2MPS(double);

      /**
       * @brief calling this function will multiply the rotations per second of the
       * motor by the set ratio of RPM2MPS set in the SetRPM2MPS method
       * 
       * @return Meters per second
       */
      double GetRate();

      /**
       * @brief Sets the Motor RPM using frc::BangBangController
       * 
       * @param RPM 
       */
      void SetRPMBB(double);

      /**
       * @brief Sets the Motor RPM using frc2::PIDController
       * 
       * @param RPM
       */
      void SetRPMpid(double);

    private:
      double m_Ratio = 1;
      frc::BangBangController m_controller;
      frc2::PIDController m_PID{1 , 0 , 0};
  }; // class NeoMotor
} // namespace FRC7110