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
#include "pros/screen.h" // IWYU pragma: keep
#include <algorithm> // IWYU pragma: keep
#include <cstdio>
#include <cmath>
#include <cstdint>

extern int auton;
bool competitionInitialize = false;
bool opControl = false;

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
pros::MotorGroup leftMotors({-2, -3, -4}, pros::MotorGearset::blue);  // left motor group
pros::MotorGroup rightMotors({8, 9, 10}, pros::MotorGearset::blue);   // right motor group

// lift - cascade (port 12, reversed) permanent motor
// ptoMotor switches between intake and cascade
pros::Motor liftMotor(-12, pros::MotorGearset::green);

// pivot (claw pitch, port 15) and flip (claw rotation, port 16 reversed)
pros::Motor pivotMotor(15);
pros::Motor flipMotor(-16);

enum class ArmState {STOW, LOAD, PICKUP, SCORING, SCORED};
ArmState armState = ArmState::LOAD;

// pneumatics
pros::adi::Pneumatics claw('A', false, true);

// TODO: tune all of these once the mechanism is built out
// The bot powers on in the load pose, and initialize() tares every arm motor,
// so each motor's 0 is its load position.
constexpr double CASCADE_LOAD_POS = 0;
constexpr double CASCADE_STOW_POS = -200;       // placeholder: fully down, below load
constexpr double CASCADE_PICKUP_POS = -200 ;
constexpr double CASCADE_TRANSITION_POS = 900;
constexpr double PIVOT_DOWN_POS = 0;            // load (power-on pose)
constexpr double PIVOT_OUT_POS = -540 * 1.1;    // pickup / scoring
constexpr double CLAW_UP_POS = 0;
constexpr double CLAW_DOWN_POS = 360 * 1.1;
bool loadPickupTransition = false;

// move the cascade to a position; hold still while the PTO is mid-switch
// (maxRpm / moveToPercent live in globals.hpp)
static void liftTo(double position) {
    if (ptoReady()) moveToPercent(liftMotor, position, 100);
    else liftMotor.brake();
}



// game color (0 for red, 1 for blue, -1 for none)
int gameColor = -1;

// Inertial Sensor (port 16)
pros::Imu imu(16);

// tracking wheels
// vertical odometry rotation sensor, port 7, reversed
pros::Rotation verticalEnc(-7);
// horizontal odometry rotation sensor, port 1
pros::Rotation horizontalEnc(1);
// horizontal tracking wheel. 2" diameter, 4" offset behind center (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, 2, -4);
// vertical tracking wheel. 2" diameter, 0.15" offset right of center (positive)
lemlib::TrackingWheel vertical(&verticalEnc, 2, 0.15);

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
        auton = 0; // default if no file is found
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
    pivotMotor.tare_position();
    flipMotor.tare_position();

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
            pros::lcd::print(7, "PTO mode: %s", ptoMode() == PtoMode::COMBINED ? "22W" : "split");

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
    std::get<1>(autons[auton])();
}

// grid of scoring heights (raw liftMotor degrees), listed lowest -> highest
// in the order reached while physically rising. TODO: measure real values.
constexpr double SCORE_GRID[] = {0, 400, 800, 1200, 1600};
constexpr int SCORE_GRID_COUNT = sizeof(SCORE_GRID) / sizeof(SCORE_GRID[0]);
constexpr int RISE_SIGN = (SCORE_GRID[SCORE_GRID_COUNT - 1] > SCORE_GRID[0]) ? 1 : -1;

constexpr double GRID_GRACE = 15;          // degrees past a rung still counted as "that" rung - TODO: tune
constexpr uint32_t TAP_THRESHOLD_MS = 200; // release before this = "quick tap" - TODO: tune

int gridFloorIndex(double pos) {
    double signedPos = pos * RISE_SIGN;
    int idx = 0;
    for (int i = 0; i < SCORE_GRID_COUNT; i++) {
        if (SCORE_GRID[i] * RISE_SIGN <= signedPos) idx = i;
        else break;
    }
    return idx;
}

double computeSnapTarget(double pos, int startIdx, uint32_t holdMs) {
    if (holdMs < TAP_THRESHOLD_MS) {
        return SCORE_GRID[std::min(startIdx + 1, SCORE_GRID_COUNT - 1)];
    }
    int floorIdx = gridFloorIndex(pos);
    double signedPos = pos * RISE_SIGN;
    double signedFloor = SCORE_GRID[floorIdx] * RISE_SIGN;
    if (signedPos - signedFloor <= GRID_GRACE) return SCORE_GRID[floorIdx];
    return SCORE_GRID[std::min(floorIdx + 1, SCORE_GRID_COUNT - 1)];
}

void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    printf("\nDriver Control Started\n");
    opControl = true;

    while (true) {
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
        static ArmState lastArmState = ArmState::LOAD;
        static uint32_t r2PressStart = 0;
        static int scoreStartIdx = 0;
        static double scoreTarget = 0;

        // intake
        if (r1Held) {
            intake(127);
        } else if (l1Held) {
            intake(-127);
        } else {
            stopIntake();
        }


        // R2: hold to score from any state; releasing it moves SCORING -> SCORED
        if (r2Held) armState = ArmState::SCORING;
        else if (armState == ArmState::SCORING) armState = ArmState::SCORED;

        // A: cancel whatever's happening and go to load.
        // B: go to pickup; press again to go back to load.
        if (aPressed) armState = ArmState::LOAD;
        else if (bPressed) armState = (armState == ArmState::PICKUP) ? ArmState::LOAD : ArmState::PICKUP;
        // Y: cancel whatever's happening and go to stow.
        if (yPressed) armState = ArmState::STOW;

        if (lastArmState == ArmState::LOAD && armState == ArmState::PICKUP) {
            loadPickupTransition = true;
        }

        // grid bookkeeping: fires once on entry to SCORING, once on the
        // SCORING -> SCORED transition
        if (armState == ArmState::SCORING && lastArmState != ArmState::SCORING) {
            scoreStartIdx = gridFloorIndex(liftMotor.get_position());
            r2PressStart = pros::millis();
        }
        if (armState == ArmState::SCORED && lastArmState == ArmState::SCORING) {
            scoreTarget = computeSnapTarget(liftMotor.get_position(), scoreStartIdx, pros::millis() - r2PressStart);
        }
        lastArmState = armState;



        // PTO: go combined (22W) once the cascade passes the threshold, and stay
        // combined until an intake button is pressed. Otherwise stay split (11W).
        if ((liftMotor.get_position() > PTO_SPLIT_MAX_POS) || (ptoMode() == PtoMode::COMBINED && (!r1Held && !l1Held))) {
            ptoRequest(PtoMode::COMBINED);
        }
        else {
            ptoRequest(PtoMode::SPLIT);
        }
        ptoUpdate();


        switch (armState) {
            case ArmState::STOW:
                moveToPercent(pivotMotor, PIVOT_OUT_POS, 100);
                if (fabs(pivotMotor.get_position() - PIVOT_OUT_POS) < 20) {
                    liftTo(CASCADE_STOW_POS);
                } else {
                    liftMotor.brake(); // hold the cascade until the pivot has cleared
                }
                break;
            case ArmState::LOAD:
                liftTo(CASCADE_LOAD_POS);
                if (fabs(liftMotor.get_position() - CASCADE_LOAD_POS) < 20) {
                    moveToPercent(pivotMotor, PIVOT_DOWN_POS, 100);
                }
                break;
            case ArmState::PICKUP:
                if (loadPickupTransition) {
                    liftTo(CASCADE_TRANSITION_POS);
                    if (fabs(liftMotor.get_position() - CASCADE_TRANSITION_POS) < 100) moveToPercent(pivotMotor, PIVOT_OUT_POS, 100);
                    if (fabs(liftMotor.get_position() - CASCADE_TRANSITION_POS) < 10) loadPickupTransition = false;
                } else {
                    liftTo(CASCADE_PICKUP_POS);
                    moveToPercent(pivotMotor, PIVOT_OUT_POS, 100);
                }
                break;
            case ArmState::SCORING:
                if (ptoReady()) {
                    liftMotor.move(127); // rises in either mode; the PTO switches over when it passes the threshold
                } else {
                    liftMotor.brake();   // brief pause while the piston switches
                }
                moveToPercent(pivotMotor, PIVOT_OUT_POS, 100);
                break;
            case ArmState::SCORED:
                liftTo(scoreTarget);
                moveToPercent(pivotMotor, PIVOT_OUT_POS, 100);
                break;
        }

        bool downPressed = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN);
        static bool clawDown = false;
        if (downPressed) {
            clawDown = !clawDown;
            moveToPercent(flipMotor, clawDown ? CLAW_DOWN_POS : CLAW_UP_POS, 50);
        }

        pros::delay(10);
    }
}