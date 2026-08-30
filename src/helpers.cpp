#include "globals.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cstdio>

bool intaking = false;
bool armMoving = false;

double angular_kp = 1.62;
double angular_ki = 0.25;
double angular_kd = 12;

void turnToHeadingU30(float heading, int timeout, lemlib::TurnToHeadingParams params, bool async) {
    angularController.kP = 0;
    angularController.kI = 0;
    angularController.kD = 0;
    angularController.windupRange = 5;
    chassis.turnToHeading(heading, timeout, params, async);
    angularController.kP = 1.62; // reset to default
    angularController.kI = 0.25;
    angularController.kD = 12;
    angularController.windupRange = 5;
}

void intake() {
    intakeM.move(127);
    intaking = true;
}

void moveWithVoltage(int left, int right) {
    leftMotors.move_voltage(left * 12000 / 127);
    rightMotors.move_voltage(right * 12000 / 127);
}

void stopDrive() {
    leftMotors.move_voltage(0);
    rightMotors.move_voltage(0);
}

void stopIntake() {
    intakeM.move(0);
    intaking = false;
}

// TODO: implement once this season's scoring mechanism is wired up
void score(int speed) {}

// TODO: implement once this season's arm/lift motor is wired up
void stopArm() {
    armMoving = false;
}

double distanceResetX(bool right, int wallOffset) {
    double angleDistanceX = (((right ? rightDist.get() : leftDist.get()) + (right ? 3.1 : -3.1) * sin(lemlib::degToRad(chassis.getPose().theta))) / 25.4) + 5.1;
    double finalDistanceX = angleDistanceX * cos(lemlib::degToRad(chassis.getPose().theta - wallOffset));
    return fabs(finalDistanceX);
}

double distanceResetX(bool right, double x, double heading) {
    double angleDistanceX = ((right ? x - 3.1 * sin(lemlib::degToRad(heading)) : x) / 25.4) + 4.1;
    double finalDistanceX = angleDistanceX * cos(lemlib::degToRad(heading));
    return fabs(finalDistanceX);
}

double distanceResetY(int wallOffset) {
    double angleDistanceY = ((backDist.get() + 3.5 * sin(lemlib::degToRad(chassis.getPose().theta))) / 25.4) + 7;
    double finalDistanceY = angleDistanceY * cos(lemlib::degToRad(chassis.getPose().theta - wallOffset));
    double pitchCorrected = finalDistanceY * cos(lemlib::degToRad(imu.get_pitch()));
    return fabs(pitchCorrected);
}

double distanceResetY(int y, double heading) {
    double angleDistanceY = (y / 25.4) + 4.8;
    double finalDistanceY = angleDistanceY * cos(lemlib::degToRad(heading));
    return fabs(finalDistanceY);
}

// Shared by tune_kp/tune_ki/tune_kd: resets pose to (0,0,0), performs one
// turnToHeading toward `target`, and counts how many times the heading
// error changes sign (an oscillation), also reporting the final error.
namespace {
struct TuneResult { int oscillation; int finalError; };

TuneResult measureOscillations(int target) {
    chassis.setPose(0, 0, 0);
    int oscillation = 0;
    int currError = lemlib::angleError(target, chassis.getPose().theta, false);
    int prevError;
    chassis.turnToHeading(target, 1000);
    for (int i = 0; i < 100; i++) {
        prevError = currError;
        currError = lemlib::angleError(target, chassis.getPose().theta, false);
        pros::delay(10);
        if (currError * prevError < 0) oscillation++;
    }
    return {oscillation, currError};
}
} // namespace

void tune_kp(int target, int& oscillation) {
    auto result = measureOscillations(target);
    oscillation = result.oscillation;
    if (oscillation == 0) angular_kp += 0.1;
    else if (oscillation > 1) angular_kp -= 0.05 * oscillation;
}

void tune_ki(int target, int& oscillation) {
    auto result = measureOscillations(target);
    oscillation = result.oscillation;
    if (result.finalError > 1 && angular_ki < 0.3) angular_ki += 0.01;
    else if (oscillation > 0) angular_ki -= 0.005 * oscillation;
}

void tune_kd(int target, int& oscillation) {
    auto result = measureOscillations(target);
    oscillation = result.oscillation;
    if (oscillation > 0) angular_kd += 0.5;
    else if (result.finalError > 1.5) angular_kd -= 0.1;
}

void pidTuneAngular(int target) {
    int oscillation = 0;
    angular_kp = 3;
    angular_ki = 0;
    angular_kd = 0;
    do {
        tune_kp(target, oscillation);
    } while (oscillation != 1);
    do {
        tune_kd(target, oscillation);
    } while (oscillation > 0);
    do {
        tune_ki(target, oscillation);
    } while (oscillation > 0 && lemlib::angleError(target, chassis.getPose().theta, false) > 1);
}