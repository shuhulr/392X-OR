#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/logger/telemetrySink.hpp" // IWYU pragma: keep

#include "liblvgl/llemu.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.hpp" // IWYU pragma: keep
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "autons.h"    // IWYU pragma: keep
#include "globals.hpp"
#include "pto.hpp"
#include "pivot.hpp"
#include "pros/screen.h" // IWYU pragma: keep
#include <algorithm> // IWYU pragma: keep
#include <cstdio>
#include <cmath>
#include <cstdint>

extern int auton;
bool competitionInitialize = false;
bool opControl = false;

// TEMPORARY CASCADE TEST MODE: set true to bypass normal arm controls and
// command the cascade motor + PTO motor at full voltage in COMBINED mode.
// Set back to false before normal driver practice or competition.
constexpr bool CASCADE_TEST_MODE = false;


bool startupWindDone = false;
bool startupWindHold = false;


// screen task flag
bool screenTaskRunning = true;

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
// NOTE: some drivetrain motors are 5.5W while the rest are 11W.
// Mixing wattages in one group is a known trouble spot - LemLib issue
// #186 documents 5.5W motors not always being ratioed the same as 11W ones
// in a group. Keep an eye on drift/uneven power between the front and rear
// motors; if you see it, that's likely why.
pros::MotorGroup leftMotors({-16, -17, -18}, pros::MotorGearset::blue);  // left motor group
pros::MotorGroup rightMotors({12, 13, 14}, pros::MotorGearset::blue);   // right motor group

// lift - cascade (port 12, reversed) permanent motor
// ptoMotor switches between intake and cascade
pros::Motor liftMotor(21, pros::MotorGearset::green);

// pivot (claw pitch, port 6) and flip (claw rotation, port 16 reversed)
pros::Motor pivotMotor(1);
pros::Motor flipMotor(4, pros::MotorGearset::green);

// Dedicated pivot feedback sensor on port 9. Negative port reverses its
// reported direction, so pivot-up should produce increasing position.

enum class ArmState {STOW, LOAD, PICKUP, SCORING, SCORED};
ArmState armState = ArmState::LOAD;

// pneumatics
pros::adi::Pneumatics claw('A', true, false);
pros::adi::Pneumatics toggler('H', false, true);

// TODO: tune all of these once the mechanism is built out


constexpr double CLAW_UP_POS = 0;
constexpr double CLAW_DOWN_POS = 180;
bool clawDown = false;
double flipTarget = CLAW_UP_POS;

constexpr double FLIP_KP = 100.0;
constexpr double FLIP_TOLERANCE = 5.0;
constexpr double FLIP_MAX_VOLTAGE = 7000.0;
bool loadPickupTransition = false;
bool loadTransitionReached = false;

// move the cascade to a position; hold still while the PTO is mid-switch
// (maxRpm / moveToPercent live in globals.hpp)
void liftTo(double position) {
    if (ptoReady()) moveToPercent(liftMotor, position, 100);
    else liftMotor.brake();
}

void windCascadeAtMatchStart(bool holdAfterWinding = false) {
    if (startupWindDone) return;

    // Keep the pivot/claw in its physical LOAD pose while winding the string.
    ptoRequest(PtoMode::COMBINED);
    const uint32_t start = pros::millis();
    while (!ptoReady() || ptoMode() != PtoMode::COMBINED) {
        ptoRequest(PtoMode::COMBINED);
        ptoUpdate();
        liftMotor.brake();
        if (pros::millis() - start > 1500) {
            liftMotor.brake();
            return; // fail safe if the PTO does not engage
        }
        pros::delay(10);
    }

    liftMotor.move_absolute(CASCADE_STOW_POS, maxRpm(liftMotor));
    const uint32_t windStart = pros::millis();
    while (std::fabs(liftMotor.get_position() - CASCADE_STOW_POS) > 20 &&
           pros::millis() - windStart < 3000) {
        ptoRequest(PtoMode::COMBINED);
        ptoUpdate();
        pros::delay(10);
    }
    liftMotor.brake();
    startupWindDone = true;
    startupWindHold = holdAfterWinding;
}




// game color (0 for red, 1 for blue, -1 for none)
int gameColor = -1;

// Inertial Sensor (port 7)
pros::Imu imu(7);

// tracking wheels
// vertical odometry rotation sensor, port 7, reversed
pros::Rotation verticalEnc(-15);
// horizontal odometry rotation sensor, port 10, reversed
pros::Rotation horizontalEnc(10);
// horizontal tracking wheel. 2" diameter, 4" offset behind center (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, 2, -3.8);
// vertical tracking wheel. 2" diameter, 0.15" offset right of center (positive)
lemlib::TrackingWheel vertical(&verticalEnc, 2, -0.3);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              12,                        // track width (in)
                              lemlib::Omniwheel::NEW_275,  // 2.75" omnis
                              450,                          // drivetrain rpm
                              8                             // horizontal drift
);

// lateral motion controller
lemlib::ControllerSettings linearController(6,  // kP
                                            0.24,  // kI
                                            12,    // kD
                                            2,     // anti windup
                                            0.5,   // small error range (in)
                                            100,   // small error timeout (ms)
                                            2,     // large error range (in)
                                            500,   // large error timeout (ms)
                                            15     // max acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(angular_kp,
                                             angular_ki,
                                             angular_kd,
                                             5,    // anti windup
                                             1,    // small error range (deg)
                                             75,   // small error timeout (ms)
                                             3,    // large error range (deg)
                                             250,  // large error timeout (ms)
                                             0     // max acceleration (slew)
);

// secondary angular controller, used for tighter U40-class turns
lemlib::ControllerSettings angularControllerU40(4.7,
                                             0.28,
                                             20.5,
                                             4,
                                             1,
                                             75,
                                             3,
                                             250,
                                             0
);

// sensors for odometry.
// RCL relies on the IMU for heading and distance sensors for X/Y, so the
// tracking wheels are left out of the odometry chain while it's enabled.
// If RCL is disabled, fall back to the tracking wheels for X/Y instead.
#if RCL_ENABLED
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);
#else
lemlib::OdomSensors sensors(&vertical, nullptr, &horizontal, nullptr, &imu);
#endif

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(13, 13, 1);
// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(13, 13, 1);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, angularControllerU40, sensors, &throttleCurve, &steerCurve);

// distance sensors
pros::Distance leftDist(18);
pros::Distance rightDist(5);
pros::Distance backDist(17);

#if RCL_ENABLED
// RCL wall-distance sensors: (horizontal offset in, vertical offset in, sensor heading relative to bot in deg)
RclSensor rightRcl(&rightDist, 3.1, 5.1, 90);
RclSensor leftRcl(&leftDist, -3.1, 5.1, 270);
RclSensor backRcl(&backDist, -3.25, -7, 180);
RclTracking RclMain(&chassis);

// Field obstacles (goals, loaders, walls, etc.) that RCL sensor rays should
// ignore go here. None defined yet for this season - add them once the
// field layout is finalized, e.g.:
//   Circle_Obstacle someGoal(x, y, radius);
#endif

void updateAutoFile() {
    FILE* file = fopen("/usd/auto.txt", "r+");
    if (file != NULL) {
        fseek(file, 0, SEEK_SET);
        fprintf(file, "%d", auton);
        fclose(file);
    } else {
        // If file doesn't exist, create one
        file = fopen("/usd/auto.txt", "w");
        if (file != NULL) {
            fprintf(file, "%d", auton);
            fclose(file);
        }
    }
}

// cycle to the previous/next auton and persist the choice to the SD card
void leftScreenButton() {
    auton = (auton - 1 + autons.size()) % autons.size();
    updateAutoFile();
}

void rightScreenButton() {
    auton = (auton + 1) % autons.size();
    updateAutoFile();
}

void displayImage();

void initialize() {
    
    pivotMotor.set_encoder_units(pros::v5::MotorEncoderUnits::degrees);
    flipMotor.set_encoder_units(pros::v5::MotorEncoderUnits::degrees);
#if RCL_ENABLED
    RclMain.startTracking();
#endif

    FILE* file = fopen("/usd/auto.txt", "r");
    if (file != NULL) {
        char buf[3]; // two digits + null terminator
        fread(buf, 1, 2, file);
        buf[2] = '\0'; // tells stoi where the string ends
        auton = std::stoi(buf);
        printf("Loaded Auton: %s\n", buf);
        fclose(file);
    } else {
        auton = 1; // default if no file is found
        printf("SD Card not found, defaulting to auton 0\n");
    }

    displayImage();

    chassis.calibrate(); // calibrate sensors

    // TODO: update to this season's actual starting coordinates
    chassis.setPose(0, 0, 0);

#if RCL_ENABLED
    RclMain.setRclPose(chassis.getPose());
    RclMain.updateBotPose(&backRcl);
    RclMain.updateBotPose(&leftRcl);
#endif

    // hold position when no voltage is applied, instead of coasting/falling
    liftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // zero the arm motors so every position constant is relative to the load
    // pose - the bot must be physically in the load pose when powered on
    liftMotor.tare_position();
    pivotMotor.tare_position(-5); // retained for diagnostics; pivot feedback lives in pivot.cpp
    flipMotor.tare_position();
    pivotInitialize(); // robot must be physically in LOAD pose here
    pivotMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    flipMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


    // thread for brain screen display and position logging
    pros::Task screenTask([&]() {
        while (screenTaskRunning) {
#if RCL_ENABLED
            pros::lcd::print(0, "X: %.4f", RclMain.getRclPose().x);
            pros::lcd::print(1, "Y: %.4f", RclMain.getRclPose().y);
#else
            pros::lcd::print(0, "X: %.4f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %.4f", chassis.getPose().y);
#endif
            pros::lcd::print(2, "Theta: %.3f", chassis.getPose().theta);
            pros::lcd::print(3, "auton index: %d", auton);
            pros::lcd::print(4, "%s", std::get<0>(autons[auton]).c_str());
            pros::lcd::print(5, "Left: %.1f  Right: %.1f", leftMotors.get_temperature(), rightMotors.get_temperature());
            pros::lcd::print(6, "PTO: %.1f  Lift: %.1f", ptoMotor.get_temperature(), liftMotor.get_temperature());
            //pros::lcd::print(7, "PTO mode: %s", ptoMode() == PtoMode::COMBINED ? "22W" : "split");
            //pros::lcd::print(7, "liftMotor position: %.2lf", liftMotor.get_position());

            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());

            pros::delay(50); // delay to save resources
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {
    competitionInitialize = true;
}

void autonomous() {
    windCascadeAtMatchStart(false);
    ptoMotor.tare_position();
    std::get<1>(autons[auton])();
}

// Scoring levels are generated from one constant spacing.
// IMPORTANT: Tune SCORE_SPACING to the measured motor degrees
// for one pin + cup stack.
constexpr double SCORE_SPACING = 400;
constexpr double SCORE_MIN_POS = 0;
constexpr double SCORE_MAX_POS = 1600;


void opcontrol() {
    chassis.setPose(0, 0, 0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    printf("\nDriver Control Started\n");
    opControl = true;
    windCascadeAtMatchStart(true);

    while (true) {
        // Isolated cascade test: stop drivetrain, engage the PTO, then run both
        // cascade motors at full voltage. Wait safely while the PTO is switching.
        if (CASCADE_TEST_MODE) {
            chassis.arcade(0, 0);
            ptoRequest(PtoMode::COMBINED);
            ptoUpdate();

            if (ptoReady() && ptoMode() == PtoMode::COMBINED) {
                liftMotor.move_voltage(12000);
                // In COMBINED mode, ptoUpdate makes ptoMotor follow the lift
                // motor's applied voltage using the configured PTO_FOLLOW_SIGN.
                ptoUpdate();
            } else {
                liftMotor.move(0);
                ptoMotor.move(0);
            }

            pros::delay(10);
            continue; // bypass all normal driver-control commands in test mode
        }

        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // rajeev drive
        chassis.arcade(rightY, 0.95 * leftX);
        // shuhul drive
        // chassis.arcade(leftY, rightX);

          

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) claw.toggle();

        bool aPressed = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A);
        bool bPressed = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B);
        bool yPressed = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y);
        bool r2Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool r1Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool l1Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool l2Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        static ArmState lastArmState = ArmState::LOAD;
        static double scoreTarget = CASCADE_LOAD_POS;
        static bool scoringTransitionActive = false;

        // intake
        if (r1Held) {
            intake(127);
        } else if (l1Held) {
            intake(-127);
        } else {
            stopIntake();
        }


        // Any arm command exits the initial post-winding hold and restores normal controls.
        if (aPressed || bPressed || r2Held) startupWindHold = false;

        // R2: hold to score from any state; releasing it moves SCORING -> SCORED
        if (r2Held) armState = ArmState::SCORING;
        else if (armState == ArmState::SCORING) armState = ArmState::SCORED;

        // A: cancel whatever's happening and go to load.
        // B: go to pickup; press again to go back to load.
        if (aPressed) {
            armState = ArmState::LOAD;
            loadTransitionReached = false;
        } else if (bPressed) armState = (armState == ArmState::PICKUP) ? ArmState::LOAD : ArmState::PICKUP;
        

        if (lastArmState == ArmState::LOAD && armState == ArmState::PICKUP) {
            loadPickupTransition = true;
        } else if (armState != ArmState::PICKUP) {
            loadPickupTransition = false; // cancelled: don't leave it armed
        }

        // On R2 press, start the coordinated scoring transition.
        // On release, advance the stored scoring target by one stack spacing.
        if (armState == ArmState::SCORING &&
            lastArmState == ArmState::LOAD) {
            scoringTransitionActive = pivotBeginScoringTransition();
        }

        if (armState == ArmState::SCORED &&
            lastArmState == ArmState::SCORING) {

            scoreTarget = std::clamp(
                scoreTarget + SCORE_SPACING,
                SCORE_MIN_POS,
                SCORE_MAX_POS
            );

            scoringTransitionActive = false;
            pivotCancelScoringTransition();

        } else if (armState != ArmState::SCORING) {
            scoringTransitionActive = false;
            pivotCancelScoringTransition();
        }

        // Reset the stored scoring level when returning to LOAD.
        if (aPressed) {
            scoreTarget = CASCADE_LOAD_POS;
        }

        lastArmState = armState;

        if (yPressed) {
            toggler.toggle();
        }


        // PTO: force COMBINED while R2 is held so the cascade has both motors
        // available from the start of scoring. Otherwise preserve the normal
        // height-based / intake-based PTO behavior.
        if (r2Held ||
            (liftMotor.get_position() > PTO_SPLIT_MAX_POS) ||
            (ptoMode() == PtoMode::COMBINED && (!r1Held && !l1Held))) {
            ptoRequest(PtoMode::COMBINED);
        } else {
            ptoRequest(PtoMode::SPLIT);
        }
        ptoUpdate();


        if (startupWindHold) {
            liftTo(CASCADE_STOW_POS);
            pivotTo(PIVOT_DOWN_POS, 50);
        } else switch (armState) {
            case ArmState::STOW:
                pivotTo(PIVOT_OUT_POS, 100);
                if (fabs(pivotPositionDegrees() - PIVOT_OUT_POS) < 20) {
                    liftTo(CASCADE_STOW_POS);
                } else {
                    liftMotor.brake(); // hold the cascade until the pivot has cleared
                }
                break;
            case ArmState::LOAD:
                if (!loadTransitionReached) {
                    liftTo(CASCADE_TRANSITION_POS);
                }
                if (fabs(liftMotor.get_position() - CASCADE_TRANSITION_POS) < 100) {
                    loadTransitionReached = true;
                    pivotTo(PIVOT_DOWN_POS, 100);
                    liftTo(CASCADE_LOAD_POS);
                }
                break;
            case ArmState::PICKUP:
                if (loadPickupTransition) {
                    liftTo(CASCADE_TRANSITION_POS);
                    if (fabs(liftMotor.get_position() - CASCADE_TRANSITION_POS) < 100) {
                        pivotTo(PIVOT_PICKUP_POS, 100);
                    }
                    if (fabs(pivotPositionDegrees() - PIVOT_OUT_POS) < 20) {
                        loadPickupTransition = false; // pivot is out, now it's safe to lower
                    }
                } else {
                    liftTo(CASCADE_PICKUP_POS);
                    pivotTo(PIVOT_PICKUP_POS, 100);
                }
                break;
            case ArmState::SCORING: {
                if (scoringTransitionActive) {
                    PivotTransitionStatus status = pivotUpdateScoringTransition();
                    if (status == PivotTransitionStatus::RUNNING) {
                        break; // pivot.cpp is commanding both lift and pivot this loop
                    }
                    // COMPLETE or TIMED_OUT: hand off immediately to regular scoring.
                    scoringTransitionActive = false;
                }

                // Regular scoring behavior. This is also the fallback if the
                // coordinated transition reaches its timeout before the pivot clears.
                if (ptoReady()) {
                    liftMotor.move(127);
                } else {
                    liftMotor.brake();
                }
                pivotTo(PIVOT_OUT_POS, 100);
                break;
            }
            case ArmState::SCORED:
                pivotTo(PIVOT_OUT_POS, 100);
                if (l2Held) {
                    liftTo(CASCADE_PICKUP_POS);
                } else {
                    liftMotor.brake();
                }
                break;
        }

        bool downPressed = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN);

        if (downPressed && armState != ArmState::LOAD) {
            clawDown = !clawDown;
            flipTarget = clawDown ? CLAW_DOWN_POS : CLAW_UP_POS;

            printf("Flip target: %.2f\n", flipTarget);
        }

        double flipError = flipTarget + flipMotor.get_position();

        if (fabs(flipError) <= FLIP_TOLERANCE) {
            flipMotor.move_voltage(0);
        } 
        else {
            double flipVoltage = -flipError * FLIP_KP;

            flipVoltage = std::clamp(
                flipVoltage,
                -FLIP_MAX_VOLTAGE,
                FLIP_MAX_VOLTAGE
            );

            flipMotor.move_voltage(static_cast<int32_t>(flipVoltage));
        }

        printf("Position: %.2f | Target: %.2f | Error: %.2f\n",
            flipMotor.get_position(),
            flipTarget,
            flipError);

        
        pros::delay(10);
    }
}   