#pragma once

#include <ctre/phoenix.h>
#include <wpi/numbers>

namespace FRC7110{
  class CANcode : CANCoder{
    public:
      explicit CANcode(const int id, const double offset) : CANCoder(id), m_offset(offset){};
      
      void SetDistancePerPulse(const double DPP){}

      double Get() {

        double degPos = GetPosition() - m_offset;

        double radPos = (degPos/360) * 2 * wpi::numbers::pi - wpi::numbers::pi;

        return radPos;
      }

      private:

        const double m_offset;
        
    };
} // namespace FRC7110