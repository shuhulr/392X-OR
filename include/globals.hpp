// globals.hpp
#pragma once
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "RclTracking.hpp"

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
extern RclSensor rightRcl;
extern RclSensor leftRcl;
extern RclSensor backRcl;
extern RclTracking RclMain;


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
