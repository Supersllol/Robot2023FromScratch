#pragma once
#define _USE_MATH_DEFINES
#include <stdint.h>
#include <math.h>
#include <cmath>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <units/voltage.h>
#define MOTOR_2_DRIVES
namespace DriveConstant
{
    constexpr double kMotorPulsesPerTurn    = 42.0;
    constexpr double kGearBoxRatio          = 10.75;
    constexpr int kWheelPulsesPerTurn       = 42;
    constexpr int DriveDroitMaitreID = 11;
    constexpr int DriveDroitEsclaveID = 12;
    constexpr int DriveGaucheMaitreID = 10;
    constexpr int DriveGaucheEsclaveID = 13;
    constexpr double kWheelDiameterM = 0.149352;
    constexpr auto kWheelCirconfM        = (kWheelDiameterM * M_PI);
    constexpr bool kGyroReversed            = false;
    
    constexpr auto ks = 0.168_V;
    constexpr auto kv = 2.86 * 1_V * 1_s / 1_m;
    constexpr auto ka = 0.321 * 1_V * 1_s * 1_s / 1_m;
    extern const frc::DifferentialDriveKinematics kDriveKinematics;
    constexpr auto kTrackwidth = 0.590_m;
    constexpr double kPDriveVelocity = 2;
    constexpr auto kMaxSpeed = 3_mps;
    constexpr auto kMaxAcceleration = 1_mps_sq;
    
    constexpr auto kRamseteB = 2.0 * 1_rad * 1_rad / (1_m * 1_m);
    constexpr auto kRamseteZeta = 0.7 / 1_rad;
    enum IdleMode {
        COAST = 0,
        BRAKE = 1
    };


}

namespace OIConstant
{
    constexpr int ThrottleStickID = 0;
    constexpr int TurnStickID =1;
    constexpr int CoPilotControllerID = 2; 
    constexpr int CoPilot_A_Button = 1;
    constexpr int CoPilot_B_Button = 2;
    constexpr int CoPilot_X_Button = 3;
    constexpr int CoPilot_Y_Button = 4;
    constexpr int CoPilot_LB_Button = 5;
    constexpr int CoPilot_RB_Button = 6;
    constexpr int CoPilot_Back_Button = 7;
    constexpr int CoPilot_Start_Button = 8;
    constexpr int CoPilot_LPush_Button = 9;
    constexpr int CoPilot_RPush_Button = 10;
}
