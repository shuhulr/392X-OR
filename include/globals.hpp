// globals.hpp
#pragma once
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"

// ============================================================
// FEATURE FLAGS
// ============================================================
// Set to 0 to fully disable the RCL (distance-sensor wall) localization
// system at compile time. When off, none of the RCL sensors/task get
// built, and odometry falls back to the tracking wheels instead.
#define RCL_ENABLED 1

#if RCL_ENABLED
#include "RclTracking.hpp"
#endif

// Declare chassis so other files can use it
extern lemlib::Chassis chassis;
extern lemlib::ControllerSettings linearController;
extern lemlib::ControllerSettings angularController;
extern lemlib::ControllerSettings angularControllerU30;
extern pros::Controller controller;
extern pros::Motor intakeLeft;
extern pros::Motor intakeRight;
extern pros::MotorGroup intakeM;
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::adi::Pneumatics matchloader;
extern pros::Distance leftDist;
extern pros::Distance rightDist;
extern pros::Distance backDist;
extern pros::Imu imu;

#if RCL_ENABLED
extern RclSensor rightRcl;
extern RclSensor leftRcl;
extern RclSensor backRcl;
extern RclTracking RclMain;
#endif

// helpers.cpp
extern void turnToHeadingU30(float heading, int timeout, lemlib::TurnToHeadingParams params = {}, bool async = true);
extern void intake();
extern void moveWithVoltage(int left, int right);
extern void stopIntake();
extern void stopDrive();
extern void score(int speed);
extern void stopArm();
extern double distanceResetX(bool right, int wallOffset);
extern double distanceResetX(bool right, double x, double heading);
extern double distanceResetY(int wallOffset);
extern double distanceResetY(int y, double heading);
extern void pidTuneAngular(int target);
extern double angular_kp;
extern double angular_ki;
extern double angular_kd;
extern void tune_kp(int target, int& oscillation);
extern void tune_ki(int target, int& oscillation);
extern void tune_kd(int target, int& oscillation);

extern bool intaking;
extern bool armMoving;