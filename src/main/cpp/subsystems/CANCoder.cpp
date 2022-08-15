#include "subsystems/CANCoder.h"

using namespace FRC7110;

CANcode::CANcode(const int Id, const double offset) : CANCoder(Id), m_offset(offset){}

double CANcode::Get() {
        double degPos = GetPosition() - m_offset;

        double radPos = (degPos/360) * 2 * wpi::numbers::pi - wpi::numbers::pi;

        return radPos;
}