// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/numbers>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
    namespace CanIds{
    constexpr int kFrontLeftDriveMotorPort = 5;
    constexpr int kRearLeftDriveMotorPort = 7;
    constexpr int kFrontRightDriveMotorPort = 6;
    constexpr int kRearRightDriveMotorPort = 8;

    constexpr int kFrontLeftTurningMotorPort = 1;
    constexpr int kRearLeftTurningMotorPort = 3;
    constexpr int kFrontRightTurningMotorPort = 2;
    constexpr int kRearRightTurningMotorPort = 4;

    constexpr int kFrontLeftTurningEncoderPorts = 0;
    constexpr int kRearLeftTurningEncoderPorts = 2;
    constexpr int kFrontRightTurningEncoderPorts = 1;
    constexpr int kRearRightTurningEncoderPorts = 3;

    const int kPidgeonID = 0;
    } // namespace CanIds

constexpr double kFrontLeftOffset = -146.6;
constexpr double kRearLeftOffset = 138.25;
constexpr double kFrontRightOffset = -74.53;
constexpr double kRearRightOffset = 16.88;

}  // namespace DriveConstants

namespace ModuleConstants {
constexpr double kGearRatio = 1/6.75;
constexpr double kWheelDiameterMeters = 0.05092958;
constexpr double kDriveEncoderDistancePerPulse =
kGearRatio * 2 * wpi::numbers::pi * kWheelDiameterMeters;

constexpr double kPModuleTurningController = 1;
constexpr double kPModuleDriveController = 0.5;
}  // namespace ModuleConstants

namespace AutoConstants {
using radians_per_second_squared_t =
    units::compound_unit<units::radians,
                         units::inverse<units::squared<units::second>>>;

constexpr auto kMaxSpeed = units::meters_per_second_t(4.4);
constexpr auto kMaxAcceleration = units::meters_per_second_squared_t(4.4);
constexpr auto kMaxAngularSpeed = units::radians_per_second_t(3.142);
constexpr auto kMaxAngularAcceleration = units::unit_t<radians_per_second_squared_t>(3.142);

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;



extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
}  // namespace OIConstants
