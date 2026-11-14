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
int auton = 1;
extern bool screenTaskRunning;

lemlib::Pose origin(0, 0, 0);
// (0, 0, 0) is center of field facing red side



bool transferTaskOn = true;


void PIDTesting() {

    // Pivot continues moving toward LOAD during the turn.
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
    // chassis.turnToHeading(39, 2000, {}, false);


    // LATERAL
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 6, 2000, {}, false);
    pros::delay(2000);
    chassis.moveToPoint(0, 0, 2000, {.forwards=false, .maxAngularSpeed=5}, false);
    pros::delay(2000);
    chassis.setPose(0, 0, chassis.getPose().theta);
    chassis.moveToPoint(0, 12, 2000, {}, false);
    pros::delay(2000);
    chassis.moveToPoint(0, 0, 2000, {.forwards=false, .maxAngularSpeed=5}, false);
    pros::delay(2000);
    chassis.setPose(0, 0, chassis.getPose().theta);
    chassis.moveToPoint(0, 24, 2000, {}, false);
    pros::delay(2000);
    chassis.moveToPoint(0, 0, 2000, {.forwards=false, .maxAngularSpeed=5}, false);
    pros::delay(2000);
    chassis.setPose(0, 0, chassis.getPose().theta);
    chassis.moveToPoint(0, 36, 2000, {}, false);
    pros::delay(2000);
    chassis.moveToPoint(0, 0, 2000, {.forwards=false, .maxAngularSpeed=5}, false);
    pros::delay(2000);
    chassis.setPose(0, 0, chassis.getPose().theta);
    chassis.moveToPoint(0, 48, 2000, {}, false);
    pros::delay(2000);
    chassis.moveToPoint(0, 0, 2000, {.forwards=false, .maxAngularSpeed=5}, false);
    pros::delay(2000);

}

void leftSubmissive() {
}
void rightSubmissive() {
}

void leftDominant() {
    // // The robot should start physically in LOAD, matching pivotInitialize().
    // pros::Task updateTask(task);
    // chassis.setPose(0, -62.75, 180);
    // pivotInitialize();

    
    // claw.extend();
    
    // toggler.extend();
    // moveWithVoltage(-35,-35);
    // pros::delay(420);

    // // toggle 1
    // moveWithVoltage(50, 50);
    // pros::delay(400);
    // pros::Task transfer(transferTask);
    // moveWithVoltage(-35,-35);
    // pros::delay(450);

    // // toggle 2
    // moveWithVoltage(50,50);
    // pros::delay(400);
    
    // chassis.moveDistance(-6, 250, {.forwards=false}, false); 
    // chassis.moveToPoint(0, -48, 650, {.forwards=false}, false);

    // // chassis.waitUntilDone();

    // printf("Approach pose: x=%f, y=%f, theta=%f\n",
    //        chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    // claw.extend();
    // // Face the scoring location, then command the pivot straight to scoring.

    
    // chassis.turnToPoint(24, -48, 600, {.forwards=false}, false);
    // // chassis.waitUntilDone();
    // printf("%lf",pivotPositionDegrees());
    
    // transferTaskOn = false;
    // liftTo(SCORE_LOW_POS+200);


    // // Approach the scoring point while the cascade continues toward height.
    // chassis.moveDistance(-20, 600, {.forwards=false});
    // chassis.waitUntil(17);
    // chassis.cancelMotion();


    // // Final small contact/settling movement.
    // moveWithVoltage(-20, -20);
    // pros::delay(350);
    // stopDrive();
    // chassis.moveDistance(2, 300, {}, true);

    // pivotTo(PIVOT_OUT_POS, 50);
    // liftTo(SCORE_LOW_POS);
    // pros::delay(450);
    // claw.retract();
    // printf("%lf",pivotPositionDegrees());
    
    // pros::delay(200);


    // chassis.moveDistance(7, 700, {}, false);
    // liftTo(CASCADE_PICKUP_POS);
    // chassis.turnToPoint(24, -24, 600, {.forwards=false}, false);

    // pivotTo(PIVOT_OUT_POS, 100);
    
    // chassis.moveDistance(-24, 800, {.forwards=false}, true);
    // chassis.waitUntil(17);
    // chassis.cancelMotion();
    // moveWithVoltage(-20, -20);

    // pros::delay(200);
    // claw.extend();
    // pros::delay(100);

    // liftTo(SCORE_LOW_POS+650);
    // pivotTo(PIVOT_OUT_POS, 100);
    // chassis.moveToPoint(24, -24, 500, {.forwards=false}, false);
    // chassis.turnToPoint(24, -48, 600, {.forwards=false}, false);


    // //!TODO CHANGE TO POINT LATER
    // chassis.moveDistance(-20, 800, {.forwards=false});
    // chassis.waitUntil(17);
    // chassis.cancelMotion();

    // // Final small contact/settling movement.
    // moveWithVoltage(-20, -20);
    // pros::delay(400);
    // stopDrive();

    // pivotTo(PIVOT_OUT_POS, 50);
    // liftTo(SCORE_LOW_POS+300);
    // pros::delay(400);
    // claw.retract();
    // printf("%lf",pivotPositionDegrees());
    
    // pros::delay(200);


    // // last 


    // chassis.moveDistance(7, 700, {}, false);
    // liftTo(CASCADE_PICKUP_POS);
    // chassis.turnToPoint(48, -48, 600, {.forwards=false}, false);

    // pivotTo(PIVOT_OUT_POS, 100);
    
    // chassis.moveDistance(-24, 800, {.forwards=false}, true);
    // chassis.waitUntil(13);
    // chassis.cancelMotion();
    // moveWithVoltage(-20, -20);

    // pros::delay(200);
    // claw.extend();
    // pros::delay(100);

    // liftTo(SCORE_LOW_POS+800);
    // pivotTo(PIVOT_OUT_POS, 100);
    // chassis.moveToPoint(48, -48, 500, {.forwards=false}, false);
    // chassis.turnToPoint(24, -48, 600, {.forwards=false}, false);


    // //!TODO CHANGE TO POINT LATER
    // chassis.moveDistance(-20, 800, {.forwards=false});
    // chassis.waitUntil(14);
    // chassis.cancelMotion();

 
    // // Final small contact/settling movement.
    // moveWithVoltage(-20, -20);
    // pros::delay(600);
    // stopDrive();

    // pivotTo(PIVOT_OUT_POS, 50);
    // liftTo(SCORE_LOW_POS+500);
    // pros::delay(400);
    // claw.retract();
    // printf("%lf",pivotPositionDegrees());
    
    // pros::delay(200);

}

void rightDominant() {

}
// Starting poses for this season's autons go here once field setup is
// finalized, e.g.:
//   lemlib::Pose RightStart(x, y, heading);

// register each auton as {"Display Name", functionPointer}. Keep at least
// one entry so the brain screen / autonomous() never index out of bounds.
std::vector<std::tuple<std::string, void(*)()>> autons = {
    {"PID Test", PIDTesting},
    {"left dom", leftDominant},
    {"right dom", rightDominant},
    {"left sub", leftSubmissive},
    {"right sub", rightSubmissive},
    
};