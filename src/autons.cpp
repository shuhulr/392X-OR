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
int auton = 1;
extern bool screenTaskRunning;

lemlib::Pose origin(0, 0, 0);
// (0, 0, 0) is center of field facing red side

// Retained scoring targets from the old SCORE_GRID: SCORE_GRID[1] was 400.
// The higher target preserves the old +30 motor-degree offset. Calibrate on robot.
constexpr double SCORE_LOW_POS = 850;
constexpr double SCORE_HIGH_POS = 1000;

void task(void* param) {
    while (true) {
        ptoUpdate();
        pivotUpdateControl();
        pros::delay(10);
    }
}
  

void PIDTesting() {
    pros::Task updateTask(task);

    // Pivot continues moving toward LOAD during the turn.
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(30, 2000, {}, false);
    pros::delay(2000);
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(60, 2000, {}, false);
    pros::delay(2000);
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 2000, {}, false);
    pros::delay(2000);
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(120, 2000, {}, false);
    pros::delay(2000);
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(150, 2000, {}, false);
    pros::delay(2000);
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(180, 2000, {}, false);
    pros::delay(2000);
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 2000, {}, false);
    pros::delay(2000);

    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(10, 2000, {}, false);
    pros::delay(2000);
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(20, 2000, {}, false);
    pros::delay(2000);
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(30, 2000, {}, false);
    pros::delay(2000);
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(40, 2000, {}, false);


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
    pivotMotor.tare_position(); 
    

    pros::Task updateTask(task);
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
    

    pros::Task updateTask(task);
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
    // The robot should start physically in LOAD, matching pivotInitialize().
    pros::Task updateTask(task);
    chassis.setPose(0, -62.75, 180);
    pivotInitialize();
    pivotMotor.tare_position(5);

    // Initial approach.
    chassis.moveDistance(-6, 500, {.forwards=false}, false);
    chassis.moveToPoint(0, -48, 800, {.forwards=false}, false);
    // chassis.waitUntilDone();

    printf("Approach pose: x=%f, y=%f, theta=%f\n",
           chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    claw.extend();
    // Face the scoring location, then command the pivot straight to scoring.
    chassis.turnToPoint(24, -48, 1000, {.forwards=false}, false);
    // chassis.waitUntilDone();
    printf("%lf",pivotPositionDegrees());
    pivotTo(PIVOT_OUT_POS, 10);
    liftTo(SCORE_HIGH_POS);

    // Approach the scoring point while the cascade continues toward height.
    chassis.moveDistance(-20, 600, {.forwards=false});
    chassis.waitUntilDone();

    // Final small contact/settling movement.
    moveWithVoltage(-20, -20);
    pros::delay(400);
    stopDrive();

    liftTo(SCORE_LOW_POS);
    pros::delay(200);
    claw.retract();
    printf("%lf",pivotPositionDegrees());
    chassis.waitUntilDone();

    chassis.moveDistance(7, 600, {}, false);
    chassis.turnToPoint(24, -24, 600, {.forwards=false}, false);
    chassis.moveDistance(-25, 800, {.forwards=false}, false);

    // PICK UP CUP PIN //
    // implement here

    pros::delay(200);
    chassis.turnToPoint(24, -48, 600, {.forwards=false}, false);
    chassis.moveDistance(-20, 800, {.forwards=false}, false);

    moveWithVoltage(-20, -20);
    pros::delay(400);
    stopDrive();

    //SCORE HERE

    chassis.moveDistance(7, 600, {}, false);
    chassis.turnToPoint(48, -48, 600, {.forwards=false}, false);
    chassis.moveDistance(-30, 1000, {.forwards=false}, false);
    pros::delay(200);

    // PICK UP CUP PIN //
    // implement here

    chassis.turnToPoint(24, -48, 800, {.forwards=false}, false);
    chassis.moveDistance(-27, 800, {.forwards=false}, false);
    moveWithVoltage(-20, -20);
    pros::delay(400);
    stopDrive();


    // Remaining pickup / second-score sequence is still intentionally omitted
    // until the first scoring cycle is tested and its targets are calibrated.
    printf("leftDominant finished first scoring cycle.\n");
}

void rightDominant() {
    pivotMotor.tare_position(); 
    

    pros::Task updateTask(task);
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
    {"PID Test", PIDTesting},
    {"left dom", leftDominant},
    {"right dom", rightDominant},
    {"left sub", leftSubmissive},
    {"right sub", rightSubmissive},
    
};