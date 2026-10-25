#include "autons.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "lemlib/util.hpp"
#include "globals.hpp"
#include "pivot.hpp"
#include "pto.hpp"
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
// (0, 0, 0) is center of field facing red side

// Retained scoring targets from the old SCORE_GRID: SCORE_GRID[1] was 400.
// The higher target preserves the old +30 motor-degree offset. Calibrate on robot.
constexpr double SCORE_LOW_POS = 850;
constexpr double SCORE_HIGH_POS = 1000;

void ptoTask(void* param) {
    while (true) {
        ptoUpdate();
        pros::delay(50); 
    }
}

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

void leftSubmissive() {
    pivotMotor.tare_position(); 
    

    pros::Task ptoUpdateTask(ptoTask);
    chassis.setPose(0, -63, 180);


    chassis.moveDistance(-6, 300, {.forwards=false}, false);

    // toggle 1
    moveWithVoltage(40,40);
    pros::delay(300);
    moveWithVoltage(-20,20);
    pros::delay(300);

    // toggle 2
    moveWithVoltage(40,40);
    pros::delay(300);
    
    chassis.moveDistance(-6, 300, {.forwards=false}, false);    

    chassis.turnToPoint(24, -48, 250, {.forwards=false}, false);
    liftTo(SCORE_HIGH_POS);
    chassis.waitUntilDone();
    chassis.moveDistance(-23, 600, {.forwards=false}, false);

    moveWithVoltage(-20,-20);
    pros::delay(400);
    liftTo(SCORE_LOW_POS);
    pros::delay(200);

    claw.retract();


    chassis.moveToPoint(12, -48, 300, {}, true);
    liftTo(CASCADE_PICKUP_POS);
    chassis.waitUntilDone();

    chassis.turnToPoint(24, -24, 150, {.forwards=false}, false);
    chassis.moveDistance(-18, 600, {.forwards=false}, false);
    chassis.moveDistance(-9, 500, {.forwards=false, .maxSpeed=30}, true);
    chassis.waitUntil(4.5);
    claw.extend();
    chassis.waitUntilDone();

    chassis.turnToPoint(24, -48, 500, {.forwards=false}, false);
    chassis.moveToPoint(24, -40, 500, {.forwards=false}, true);
    liftTo(SCORE_HIGH_POS);
    chassis.waitUntilDone();

    moveWithVoltage(-40, -40);
    pros::delay(400);
    liftTo(SCORE_LOW_POS);
    pros::delay(100);
    claw.retract();

    chassis.moveToPoint(24, -36, 300, {}, true);
    liftTo(CASCADE_PICKUP_POS);
    chassis.waitUntilDone();

    chassis.moveToPoint(-24, -60, 1000, {.forwards=true}, false);

    chassis.turnToPoint(-24, -72, 200, {.forwards=false}, false);
    
    chassis.moveToPoint(-24, -67, 500, {.forwards=false, .maxSpeed=false}, false);
    
    claw.retract();

    chassis.moveToPoint(-24, -60, 300, {.forwards=false}, true);
    liftTo(SCORE_HIGH_POS); 
    chassis.waitUntilDone();

    chassis.turnToPoint(-24, -48, 300, {.forwards=false}, false);

    moveWithVoltage(-30,-30);
    pros::delay(300);

    liftTo(SCORE_LOW_POS);
    pros::delay(200);

    claw.retract();



    printf("done");
}
void rightSubmissive() {
    pivotMotor.tare_position(); 
    

    pros::Task ptoUpdateTask(ptoTask);
    chassis.setPose(0, -63, 180);


    chassis.moveDistance(-6, 300, {.forwards=false}, false);

    // toggle 1
    moveWithVoltage(40,40);
    pros::delay(300);
    moveWithVoltage(-20,20);
    pros::delay(300);

    // toggle 2
    moveWithVoltage(40,40);
    pros::delay(300);
    
    chassis.moveDistance(-6, 300, {.forwards=false}, false);    

    chassis.turnToPoint(-24, -48, 250, {.forwards=false}, false);
    liftTo(SCORE_HIGH_POS);
    chassis.waitUntilDone();
    chassis.moveDistance(-23, 600, {.forwards=false}, false);

    moveWithVoltage(-20,-20);
    pros::delay(400);
    liftTo(SCORE_LOW_POS);
    pros::delay(200);

    claw.retract();


    chassis.moveToPoint(-12, -48, 300, {}, true);
    liftTo(CASCADE_PICKUP_POS);
    chassis.waitUntilDone();

    chassis.turnToPoint(-24, -24, 150, {.forwards=false}, false);
    chassis.moveDistance(-18, 600, {.forwards=false}, false);
    chassis.moveDistance(-9, 500, {.forwards=false, .maxSpeed=30}, true);
    chassis.waitUntil(4.5);
    claw.extend();
    chassis.waitUntilDone();

    chassis.turnToPoint(-24, -48, 500, {.forwards=false}, false);
    chassis.moveToPoint(-24, -40, 500, {.forwards=false}, true);
    liftTo(SCORE_HIGH_POS);
    chassis.waitUntilDone();

    moveWithVoltage(-40, -40);
    pros::delay(400);
    liftTo(SCORE_LOW_POS);
    pros::delay(100);
    claw.retract();

    chassis.moveToPoint(-24, -36, 300, {}, true);
    liftTo(CASCADE_PICKUP_POS);
    chassis.waitUntilDone();

    chassis.moveToPoint(24, -60, 1000, {.forwards=true}, false);

    chassis.turnToPoint(24, -72, 200, {.forwards=false}, false);
    
    chassis.moveToPoint(24, -67, 500, {.forwards=false, .maxSpeed=false}, false);
    
    claw.retract();

    chassis.moveToPoint(24, -60, 300, {.forwards=false}, true);
    liftTo(SCORE_HIGH_POS); 
    chassis.waitUntilDone();

    chassis.turnToPoint(24, -48, 300, {.forwards=false}, false);

    moveWithVoltage(-30,-30);
    pros::delay(300);

    liftTo(SCORE_LOW_POS);
    pros::delay(200);

    claw.retract();



    printf("done");
}

void leftDominant() {
    pivotMotor.tare_position(); 
    

    pros::Task ptoUpdateTask(ptoTask);
    chassis.setPose(0, -63, 180);


    chassis.moveDistance(-6, 500, {.forwards=false}, false);

    // toggle 1
    moveWithVoltage(60,60);
    pros::delay(400);

    chassis.moveDistance(-6, 500, {.forwards=false}, false);

    // toggle 2
    moveWithVoltage(60,60);
    pros::delay(400);
    
    chassis.moveDistance(-6, 500, {.forwards=false}, false);    


    chassis.moveToPoint(0, -48, 500, {.forwards=false}, true);
    liftTo(SCORE_HIGH_POS);
    chassis.waitUntilDone();
    printf("x: %f, y: %f, theta: %f \n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    chassis.turnToPoint(24, -48, 650, {.forwards=false}, false);
    /*chassis.moveDistance(-23, 600, {.forwards=false}, false);

    moveWithVoltage(-20,-20);
    pros::delay(400);
    liftTo(SCORE_LOW_POS);
    pros::delay(200);

    claw.retract();

    /*

    chassis.moveToPoint(12, -48, 300, {}, true);
    liftTo(CASCADE_PICKUP_POS);
    chassis.waitUntilDone();

    chassis.turnToPoint(24, -24, 150, {.forwards=false}, false);
    chassis.moveDistance(-18, 600, {.forwards=false}, false);
    chassis.moveDistance(-9, 500, {.forwards=false, .maxSpeed=30}, true);
    chassis.waitUntil(4.5);
    claw.extend();
    chassis.waitUntilDone();

    chassis.turnToPoint(24, -48, 500, {.forwards=false}, false);
    chassis.moveToPoint(24, -40, 500, {.forwards=false}, true);
    liftTo(SCORE_HIGH_POS);
    chassis.waitUntilDone();

    moveWithVoltage(-40, -40);
    pros::delay(400);
    liftTo(SCORE_LOW_POS);
    pros::delay(100);
    claw.retract();

    chassis.moveToPoint(24, -36, 300, {}, true);
    liftTo(CASCADE_PICKUP_POS);
    chassis.waitUntilDone();

    chassis.turnToPoint(48, -48, 300, {.forwards=false}, true);

    chassis.moveDistance(19, 500, {.forwards=false}, true);
    chassis.moveToPoint(48, -48, 300, {.forwards=false}, false);
    chassis.waitUntil(3);
    claw.extend();
    chassis.waitUntilDone();

    liftTo(SCORE_HIGH_POS);
    chassis.turnToPoint(24, -48, 200, {.forwards=false}, true);
    chassis.moveDistance(14, 400, {.forwards=false}, true);
    moveWithVoltage(-40, 40);
    pros::delay(500);
    liftTo(SCORE_LOW_POS);
    pros::delay(150);
    claw.retract();

    printf("done");*/
}


void rightDominant() {
    pivotMotor.tare_position(); 
    

    pros::Task ptoUpdateTask(ptoTask);
    chassis.setPose(0, -63, 180);


    chassis.moveDistance(-6, 300, {.forwards=false}, false);

    // toggle 1
    moveWithVoltage(40,40);
    pros::delay(300);
    moveWithVoltage(-20,20);
    pros::delay(300);

    // toggle 2
    moveWithVoltage(40,40);
    pros::delay(300);
    
    chassis.moveDistance(-6, 300, {.forwards=false}, false);    

    chassis.turnToPoint(-24, -48, 250, {.forwards=false}, false);
    liftTo(SCORE_HIGH_POS);
    chassis.waitUntilDone();
    chassis.moveDistance(-23, 600, {.forwards=false}, false);

    moveWithVoltage(-20,-20);
    pros::delay(400);
    liftTo(SCORE_LOW_POS);
    pros::delay(200);

    claw.retract();


    chassis.moveToPoint(-12, -48, 300, {}, true);
    liftTo(CASCADE_PICKUP_POS);
    chassis.waitUntilDone();

    chassis.turnToPoint(-24, -24, 150, {.forwards=false}, false);
    chassis.moveDistance(-18, 600, {.forwards=false}, false);
    chassis.moveDistance(-9, 500, {.forwards=false, .maxSpeed=30}, true);
    chassis.waitUntil(4.5);
    claw.extend();
    chassis.waitUntilDone();

    chassis.turnToPoint(-24, -48, 500, {.forwards=false}, false);
    chassis.moveToPoint(-24, -40, 500, {.forwards=false}, true);
    liftTo(SCORE_HIGH_POS);
    chassis.waitUntilDone();

    moveWithVoltage(-40, -40);
    pros::delay(400);
    liftTo(SCORE_LOW_POS);
    pros::delay(100);
    claw.retract();

    chassis.moveToPoint(-24, -36, 300, {}, true);
    liftTo(CASCADE_PICKUP_POS);
    chassis.waitUntilDone();

    chassis.turnToPoint(-48, -48, 300, {.forwards=false}, true);

    chassis.moveDistance(19, 500, {.forwards=false}, true);
    chassis.moveToPoint(-48, -48, 300, {.forwards=false}, false);
    chassis.waitUntil(3);
    claw.extend();
    chassis.waitUntilDone();

    liftTo(SCORE_HIGH_POS);
    chassis.turnToPoint(-24, -48, 200, {.forwards=false}, true);
    chassis.moveDistance(14, 400, {.forwards=false}, true);
    moveWithVoltage(-40, 40);
    pros::delay(500);
    liftTo(SCORE_LOW_POS);
    pros::delay(150);
    claw.retract();

    printf("done");
}
// Starting poses for this season's autons go here once field setup is
// finalized, e.g.:
//   lemlib::Pose RightStart(x, y, heading);

// register each auton as {"Display Name", functionPointer}. Keep at least
// one entry so the brain screen / autonomous() never index out of bounds.
std::vector<std::tuple<std::string, void(*)()>> autons = {
    {"left dom", leftDominant},
    {"right dom", rightDominant},
    {"left sub", leftSubmissive},
    {"right sub", rightSubmissive},
    {"PID Test", PIDTesting},
};