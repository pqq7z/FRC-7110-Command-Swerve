#include "subsystems/NeoMotors.h"

using namespace hb;

NeoMotor::NeoMotor(const int& Id, rev::CANSparkMax::MotorType type) : 
rev::CANSparkMax(Id, type), rev::SparkMaxRelativeEncoder(GetEncoder()){
  rev::CANSparkMax::SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void NeoMotor::SetRPM2MPS(const double& Ratio){
  m_Ratio = Ratio;
}

double NeoMotor::GetRate() const {
  return (GetVelocity()/60) * m_Ratio;
}

void NeoMotor::SetRPMpid(double& RPM) {
  Set(m_PID.Calculate(GetVelocity(), RPM));
}