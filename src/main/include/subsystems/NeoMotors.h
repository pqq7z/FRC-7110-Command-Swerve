#pragma once

#include <rev/CANSparkMax.h>
#include <wpi/numbers>

namespace FRC7110 {
  class NeoMotor : public rev::CANSparkMax, public rev::SparkMaxRelativeEncoder{
    public:
    
      explicit NeoMotor(const int Id, rev::CANSparkMax::MotorType type) : rev::CANSparkMax(Id, type), rev::SparkMaxRelativeEncoder(GetEncoder()){}

      void SetDistancePerPulse(double DPP){}

      double GetRate() {
        double mps = (GetVelocity()/60) * 1/6.75 * 2 * wpi::numbers::pi * 0.1016;

        // mps = motor rpm /60 (to make rps) * 1/6.75 (to account for gear ratio) * 2pi * 0.1016 (for wheel size)

        return mps;
      }
  }; // class NeoMotor
} // namespace FRC7110