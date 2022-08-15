#include "subsystems/NeoMotors.h"

using namespace FRC7110;

NeoMotor::NeoMotor(const int Id, rev::CANSparkMax::MotorType type) : 
rev::CANSparkMax(Id, type), rev::SparkMaxRelativeEncoder(GetEncoder()){}

void NeoMotor::SetRPM2MPS(const double Ratio) {
  m_Ratio = Ratio;
}

double NeoMotor::GetRate() {
  double mps = (GetVelocity()/60) * m_Ratio;

  return mps;
}
