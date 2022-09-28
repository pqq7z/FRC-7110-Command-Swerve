#include "subsystems/CANCoder.h"

using namespace hb;

CANcode::CANcode(const int& Id, const double& offset = 0) : CANCoder(Id), m_offset(offset) {
  printf("CANCoder: %i, reading %5.2f\n", Id, GetAbsolutePosition());
}

double CANcode::Get() {
  return ((GetAbsolutePosition() - m_offset)/360) * 2 * wpi::numbers::pi - wpi::numbers::pi; // Returns radians
}