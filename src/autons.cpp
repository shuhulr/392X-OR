#include "autons.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "lemlib/util.hpp"
#include "globals.hpp"
#include "liblvgl/llemu.hpp" // IWYU pragma: keep
#include "pros/llemu.hpp"    // IWYU pragma: keep
#include "pros/motors.h"     // IWYU pragma: keep
#include "pros/rtos.hpp"     // IWYU pragma: keep
#include <array>   // IWYU pragma: keep
#include <cstddef> // IWYU pragma: keep
#include <cstdio>
#include <tuple>
#include <vector>

// index of the currently selected auton (cycled via the brain screen buttons)
int auton = 0;
extern bool screenTaskRunning;

lemlib::Pose origin(0, 0, 0);

void PIDTesting() {
    // chassis.setPose(0, 0, 0);
    // chassis.turnToHeading(30, 2000, {}, false);
    // pros::delay(2000);
    // chassis.setPose(0, 0, 0);
    // chassis.turnToHeading(60, 2000, {}, false);
    // pros::delay(2000);
    // chassis.setPose(0, 0, 0);
    // chassis.turnToHeading(90, 2000, {}, false);
    // pros::delay(2000);
    // chassis.setPose(0, 0, 0);
    // chassis.turnToHeading(120, 2000, {}, false);
    // pros::delay(2000);
    // chassis.setPose(0, 0, 0);
    // chassis.turnToHeading(150, 2000, {}, false);
    // pros::delay(2000);
    // chassis.setPose(0, 0, 0);
    // chassis.turnToHeading(180, 2000, {}, false);
    // pros::delay(2000);
    // chassis.setPose(0, 0, 0);
    // chassis.turnToHeading(90, 2000, {}, false);
    // pros::delay(2000);

    // chassis.setPose(0, 0, 0);
    // chassis.turnToHeading(10, 2000, {}, false);
    // pros::delay(2000);
    // chassis.setPose(0, 0, 0);
    // chassis.turnToHeading(20, 2000, {}, false);
    // pros::delay(2000);
    // chassis.setPose(0, 0, 0);
    // chassis.turnToHeading(30, 2000, {}, false);
    // pros::delay(2000);
    // chassis.setPose(0, 0, 0);
    // chassis.turnToHeading(40, 2000, {}, false);


    // LATERAL
    // chassis.setPose(0, 0, 0);
    // chassis.moveToPoint(0, 6, 2000, {}, false);
    // pros::delay(2000);
    // chassis.moveToPoint(0, 0, 2000, {.forwards=false, .maxAngularSpeed=5}, false);
    // pros::delay(2000);
    // chassis.setPose(0, 0, chassis.getPose().theta);
    // chassis.moveToPoint(0, 12, 2000, {}, false);
    // pros::delay(2000);
    // chassis.moveToPoint(0, 0, 2000, {.forwards=false, .maxAngularSpeed=5}, false);
    // pros::delay(2000);
    // chassis.setPose(0, 0, chassis.getPose().theta);
    // chassis.moveToPoint(0, 24, 2000, {}, false);
    // pros::delay(2000);
    // chassis.moveToPoint(0, 0, 2000, {.forwards=false, .maxAngularSpeed=5}, false);
    // pros::delay(2000);
    // chassis.setPose(0, 0, chassis.getPose().theta);
    // chassis.moveToPoint(0, 36, 2000, {}, false);
    // pros::delay(2000);
    // chassis.moveToPoint(0, 0, 2000, {.forwards=false, .maxAngularSpeed=5}, false);
    // pros::delay(2000);
    // chassis.setPose(0, 0, chassis.getPose().theta);
    // chassis.moveToPoint(0, 48, 2000, {}, false);
    // pros::delay(2000);
    // chassis.moveToPoint(0, 0, 2000, {.forwards=false, .maxAngularSpeed=5}, false);
    // pros::delay(2000);


    chassis.setPose(0, 0, 0);
    chassis.moveToPose(36, 36, 90, 5000, {}, false);
    printf("done");
}
// Starting poses for this season's autons go here once field setup is
// finalized, e.g.:
//   lemlib::Pose RightStart(x, y, heading);

// register each auton as {"Display Name", functionPointer}. Keep at least
// one entry so the brain screen / autonomous() never index out of bounds.
std::vector<std::tuple<std::string, void(*)()>> autons = {
    {"PID Test", PIDTesting},
};