#include "subsystems/CANCoder.h"

using namespace FRC7110;

CANcode::CANcode(const int Id, const double offset) : CANCoder(Id){
  ConfigSensorInitializationStrategy(BootToAbsolutePosition);
  ConfigMagnetOffset(offset);
}

double CANcode::Get() {
  double radPos = (GetPosition()/360) * 2 * wpi::numbers::pi - wpi::numbers::pi;
  return radPos;
}
