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



void task(void* param) {
    while (true) {
        ptoUpdate();
        pivotUpdateControl();
        /*if (!pivotRotation.is_installed()) {
            pros::lcd::print(7, "UNPLUGGEDDD");
            printf("\n\n\nUNPLUGGED\n\n\n");
        }
        else {
            pros::lcd::print(7, "plugged");
        }*/
        pros::delay(10);
    }

    
}
bool transferTaskOn = true;
void transferTask(void* param) {

    // Start the exact same scoring transition used in driver control
    bool active = pivotBeginScoringTransition();

    while (active && transferTaskOn) {

        // Same PTO behavior as driver
        ptoRequest(PtoMode::COMBINED);
        ptoUpdate();

        // Run the exact same transition logic
        PivotTransitionStatus status =
            pivotUpdateScoringTransition();

        if (status == PivotTransitionStatus::RUNNING) {
            active = true;
        } 
        else {
            active = false;

            // The transfer is DONE.
            // Do NOT keep commanding the cascade upward.
            liftMotor.brake();

            // Hold the pivot at its final position.
            pivotTo(PIVOT_OUT_POS, 100);
        }

        pros::delay(10);
    }
    pivotTo(PIVOT_OUT_POS, 100);
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

    
    claw.extend();
    
    toggler.extend();
    moveWithVoltage(-35,-35);
    pros::delay(420);

    // toggle 1
    moveWithVoltage(50, 50);
    pros::delay(400);
    pros::Task transfer(transferTask);
    moveWithVoltage(-35,-35);
    pros::delay(450);

    // toggle 2
    moveWithVoltage(50,50);
    pros::delay(400);
    
    chassis.moveDistance(-6, 250, {.forwards=false}, false); 
    chassis.moveToPoint(0, -48, 650, {.forwards=false}, false);

    // chassis.waitUntilDone();

    printf("Approach pose: x=%f, y=%f, theta=%f\n",
           chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    claw.extend();
    // Face the scoring location, then command the pivot straight to scoring.

    
    chassis.turnToPoint(24, -48, 600, {.forwards=false}, false);
    // chassis.waitUntilDone();
    printf("%lf",pivotPositionDegrees());
    
    transferTaskOn = false;
    liftTo(SCORE_LOW_POS+200);


    // Approach the scoring point while the cascade continues toward height.
    chassis.moveDistance(-20, 600, {.forwards=false});
    chassis.waitUntil(17);
    chassis.cancelMotion();


    // Final small contact/settling movement.
    moveWithVoltage(-20, -20);
    pros::delay(350);
    stopDrive();
    chassis.moveDistance(2, 300, {}, true);

    pivotTo(PIVOT_OUT_POS, 50);
    liftTo(SCORE_LOW_POS);
    pros::delay(450);
    claw.retract();
    printf("%lf",pivotPositionDegrees());
    
    pros::delay(200);


    chassis.moveDistance(7, 700, {}, false);
    liftTo(CASCADE_PICKUP_POS);
    chassis.turnToPoint(24, -24, 600, {.forwards=false}, false);

    pivotTo(PIVOT_OUT_POS, 100);
    
    chassis.moveDistance(-24, 800, {.forwards=false}, true);
    chassis.waitUntil(17);
    chassis.cancelMotion();
    moveWithVoltage(-20, -20);

    pros::delay(200);
    claw.extend();
    pros::delay(100);

    liftTo(SCORE_LOW_POS+650);
    pivotTo(PIVOT_OUT_POS, 100);
    chassis.moveToPoint(24, -24, 500, {.forwards=false}, false);
    chassis.turnToPoint(24, -48, 600, {.forwards=false}, false);


    //!TODO CHANGE TO POINT LATER
    chassis.moveDistance(-20, 800, {.forwards=false});
    chassis.waitUntil(17);
    chassis.cancelMotion();

    // Final small contact/settling movement.
    moveWithVoltage(-20, -20);
    pros::delay(400);
    stopDrive();

    pivotTo(PIVOT_OUT_POS, 50);
    liftTo(SCORE_LOW_POS+300);
    pros::delay(400);
    claw.retract();
    printf("%lf",pivotPositionDegrees());
    
    pros::delay(200);


    // last 


    chassis.moveDistance(7, 700, {}, false);
    liftTo(CASCADE_PICKUP_POS);
    chassis.turnToPoint(48, -48, 600, {.forwards=false}, false);

    pivotTo(PIVOT_OUT_POS, 100);
    
    chassis.moveDistance(-24, 800, {.forwards=false}, true);
    chassis.waitUntil(13);
    chassis.cancelMotion();
    moveWithVoltage(-20, -20);

    pros::delay(200);
    claw.extend();
    pros::delay(100);

    liftTo(SCORE_LOW_POS+800);
    pivotTo(PIVOT_OUT_POS, 100);
    chassis.moveToPoint(48, -48, 500, {.forwards=false}, false);
    chassis.turnToPoint(24, -48, 600, {.forwards=false}, false);


    //!TODO CHANGE TO POINT LATER
    chassis.moveDistance(-20, 800, {.forwards=false});
    chassis.waitUntil(14);
    chassis.cancelMotion();

 
    // Final small contact/settling movement.
    moveWithVoltage(-20, -20);
    pros::delay(600);
    stopDrive();

    pivotTo(PIVOT_OUT_POS, 50);
    liftTo(SCORE_LOW_POS+500);
    pros::delay(400);
    claw.retract();
    printf("%lf",pivotPositionDegrees());
    
    pros::delay(200);
/*
    chassis.moveDistance(13, 600, {}, false);
    chassis.turnToPoint(48, -48, 600, {.forwards=false}, false);
    liftTo(CASCADE_PICKUP_POS);
    chassis.moveDistance(-28, 1000, {.forwards=false}, false);
    moveWithVoltage(-20, -20);

    pros::delay(200);
    claw.extend();
    liftTo(SCORE_HIGH_POS);

    // PICK UP CUP PIN //
    // implement here

    chassis.turnToPoint(24, -48, 800, {.forwards=false}, false);
    liftTo(SCORE_HIGH_POS+1600);
    chassis.moveDistance(-20, 750, {.forwards=false}, false);
    moveWithVoltage(-20, -20);
    pros::delay(400);
    liftTo(SCORE_HIGH_POS);
    stopDrive();
    claw.retract();

    // Remaining pickup / second-score sequence is still intentionally omitted
    // until the first scoring cycle is tested and its targets are calibrated.
    printf("leftDominant finished first scoring cycle.\n");*/
}

void rightDominant() {
    // The robot should start physically in LOAD, matching pivotInitialize().
    pros::Task updateTask(task);
    chassis.setPose(0, -62.75, 180);
    pivotInitialize();

    
    claw.extend();
    
    toggler.extend();
    moveWithVoltage(-35,-35);
    pros::delay(420);

    // toggle 1
    moveWithVoltage(50, 50);
    pros::delay(400);
    pros::Task transfer(transferTask);
    moveWithVoltage(-35,-35);
    pros::delay(450);

    // toggle 2
    moveWithVoltage(50,50);
    pros::delay(400);
    
    chassis.moveDistance(-6, 250, {.forwards=false}, false); 
    chassis.moveToPoint(0, -48, 650, {.forwards=false}, false);

    // chassis.waitUntilDone();

    printf("Approach pose: x=%f, y=%f, theta=%f\n",
           chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    claw.extend();
    // Face the scoring location, then command the pivot straight to scoring.

    
    chassis.turnToPoint(-24, -48, 600, {.forwards=false}, false);
    // chassis.waitUntilDone();
    printf("%lf",pivotPositionDegrees());
    
    transferTaskOn = false;
    liftTo(SCORE_LOW_POS+200);


    // Approach the scoring point while the cascade continues toward height.
    chassis.moveDistance(-20, 600, {.forwards=false});
    chassis.waitUntil(17);
    chassis.cancelMotion();


    // Final small contact/settling movement.
    moveWithVoltage(-20, -20);
    pros::delay(350);
    stopDrive();
    chassis.moveDistance(2, 300, {}, true);

    pivotTo(PIVOT_OUT_POS, 50);
    liftTo(SCORE_LOW_POS);
    pros::delay(450);
    claw.retract();
    printf("%lf",pivotPositionDegrees());
    
    pros::delay(200);


    chassis.moveDistance(7, 700, {}, false);
    liftTo(CASCADE_PICKUP_POS);
    chassis.turnToPoint(-24, -24, 600, {.forwards=false}, false);

    pivotTo(PIVOT_OUT_POS, 100);
    
    chassis.moveDistance(-24, 800, {.forwards=false}, true);
    chassis.waitUntil(17);
    chassis.cancelMotion();
    moveWithVoltage(-20, -20);

    pros::delay(200);
    claw.extend();
    pros::delay(100);

    liftTo(SCORE_LOW_POS+650);
    pivotTo(PIVOT_OUT_POS, 100);
    chassis.moveToPoint(-24, -24, 500, {.forwards=false}, false);
    chassis.turnToPoint(-24, -48, 600, {.forwards=false}, false);


    //!TODO CHANGE TO POINT LATER
    chassis.moveDistance(-20, 800, {.forwards=false});
    chassis.waitUntil(17);
    chassis.cancelMotion();

    // Final small contact/settling movement.
    moveWithVoltage(-20, -20);
    pros::delay(400);
    stopDrive();

    pivotTo(PIVOT_OUT_POS, 50);
    liftTo(SCORE_LOW_POS+300);
    pros::delay(400);
    claw.retract();
    printf("%lf",pivotPositionDegrees());
    
    pros::delay(200);


    // last 


    chassis.moveDistance(7, 700, {}, false);
    liftTo(CASCADE_PICKUP_POS);
    chassis.turnToPoint(-48, -48, 600, {.forwards=false}, false);

    pivotTo(PIVOT_OUT_POS, 100);
    
    chassis.moveDistance(-24, 800, {.forwards=false}, true);
    chassis.waitUntil(13);
    chassis.cancelMotion();
    moveWithVoltage(-20, -20);

    pros::delay(200);
    claw.extend();
    pros::delay(100);

    liftTo(SCORE_LOW_POS+800);
    pivotTo(PIVOT_OUT_POS, 100);
    chassis.moveToPoint(-48, -48, 500, {.forwards=false}, false);
    chassis.turnToPoint(-24, -48, 600, {.forwards=false}, false);


    //!TODO CHANGE TO POINT LATER
    chassis.moveDistance(-20, 800, {.forwards=false});
    chassis.waitUntil(14);
    chassis.cancelMotion();

 
    // Final small contact/settling movement.
    moveWithVoltage(-20, -20);
    pros::delay(600);
    stopDrive();

    pivotTo(PIVOT_OUT_POS, 50);
    liftTo(SCORE_LOW_POS+500);
    pros::delay(400);
    claw.retract();
    printf("%lf",pivotPositionDegrees());
    
    pros::delay(200);
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