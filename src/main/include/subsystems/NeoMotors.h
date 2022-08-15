#pragma once

#include <rev/CANSparkMax.h>
#include <wpi/numbers>

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
      void SetRPM2MPS(double Ratio);

      /**
       * @brief calling this function will multiply the rotations per second of the
       * motor by the set ratio of RPM2MPS set in the SetRPM2MPS method
       * 
       * @return Meters per second
       */
      double GetRate();

    private:
      double m_Ratio = 1;
  }; // class NeoMotor
} // namespace FRC7110