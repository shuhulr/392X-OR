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


bool clawOn = false;

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
// NOTE: some drivetrain motors are 5.5W while the rest are 11W.
// Mixing wattages in one group is a known trouble spot - LemLib issue
// #186 documents 5.5W motors not always being ratioed the same as 11W ones
// in a group. Keep an eye on drift/uneven power between the front and rear
// motors; if you see it, that's likely why.
pros::MotorGroup leftMotors({-1, -6, -18}, pros::MotorGearset::blue);  // left motor group
pros::MotorGroup rightMotors({2, 15, 19}, pros::MotorGearset::blue);   // right motor group

// lift - cascade (port 12, reversed) permanent motor
// ptoMotor switches between intake and cascade
pros::Motor leftLiftMotor(-20, pros::MotorGearset::green);
pros::Motor rightLiftMotor(11, pros::MotorGearset::green);

pros::MotorGroup lift({-20, 11}, pros::MotorGearset::green);


// pivot (claw pitch, port 6) and flip (claw rotation, port 16 reversed)
pros::Motor clawMotor(-13, pros::MotorGearset::green);

// Dedicated pivot feedback sensor on port 9. Negative port reverses its
// reported direction, so pivot-up should produce increasing position.

enum class ArmState {PICKUP, RAISING, FALLING};
ArmState armState = ArmState::PICKUP;

// pneumatics
pros::adi::Pneumatics toggler('A', true, false);

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
#include "main.h"
#include <cmath>
#include <algorithm>
#include <atomic>

// V5 Distance Sensor used for lift height (reads inches)
pros::Distance liftDistance(5);

// Helper function to read distance in inches
pros::Task* liftTask = nullptr;
bool liftRunning = false; // Tracks if the task is actually alive

double getLiftPosition() {
    int mm = liftDistance.get();
    if (mm == PROS_ERR || mm >= 9999) {
        return -1.0;
    }
    return mm / 25.4;
}

void liftUpdate(double target, uint32_t timeout) {
    double liftKP = 0.8;
    double tolerance = 0.5;
    double maxSpeed = static_cast<double>(maxRpm(lift));

    uint32_t startTime = pros::millis();

    while (true) {

        if (pros::millis() - startTime >= timeout) {
            lift.brake();
            liftRunning = false;
            return;
        }

        // Motor encoder position
        double currentPosition = lift.get_position();

        double error = target - currentPosition;

        if (std::abs(error) <= tolerance) {
            lift.brake();
            liftRunning = false;
            return;
        }

        double velocity = std::clamp(
            error * liftKP,
            -maxSpeed,
            maxSpeed
        );

        lift.move_velocity(static_cast<int32_t>(velocity));

        pros::delay(20);
    }
}

void liftTo(double target, uint32_t timeout) {

    // Stop previous lift command
    if (liftRunning && liftTask != nullptr) {
        liftTask->remove();
        liftRunning = false;
    }

    // Delete old task
    if (liftTask != nullptr) {
        delete liftTask;
        liftTask = nullptr;
    }

    // Start new lift command
    liftRunning = true;

    liftTask = new pros::Task([target, timeout] {
        liftUpdate(target, timeout);
    });
}

void stopLift() {

    if (liftRunning && liftTask != nullptr) {
        liftTask->remove();
        liftRunning = false;
    }

    if (liftTask != nullptr) {
        delete liftTask;
        liftTask = nullptr;
    }

    lift.brake();
}



// game color (0 for red, 1 for blue, -1 for none)
int gameColor = -1;

// Inertial Sensor (port 7)
pros::Imu imu(12);

pros::Distance liftDist(18);
// tracking wheels
// vertical odometry rotation sensor, port 7, reversed
pros::Rotation verticalEnc(-17);
// horizontal odometry rotation sensor, port 10, reversed
pros::Rotation horizontalEnc(16);
// horizontal tracking wheel. 2" diameter, 4" offset behind center (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, 2, -3.9);
// vertical tracking wheel. 2" diameter, 0.15" offset right of center (positive)
lemlib::TrackingWheel vertical(&verticalEnc, 2, -0.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              10,                        // track width (in)
                              lemlib::Omniwheel::NEW_275,  // 2.75" omnis
                              450,                          // drivetrain rpm
                              8                             // horizontal drift
);

// lateral motion controller
lemlib::ControllerSettings linearController(7.1,  // kP
                                            0.24,  // kI
                                            12.1,    // kD
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
lemlib::ControllerSettings angularControllerU40(4.3,
                                             0.27,
                                             21.5,
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
    lift.set_encoder_units(pros::v5::MotorEncoderUnits::degrees);
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    leftLiftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    rightLiftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    
#if RCL_ENABLED
    RclMain.startTracking();
#endif

    FILE* file = fopen("/usd/auto.txt", "r");
    if (file != NULL) {
        char buf[3] = {0}; // two digits + null terminator
        size_t n = fread(buf, 1, 2, file);
        buf[n] = '\0'; // only use the bytes actually read
        auton = (n > 0) ? atoi(buf) : 1;
        if (auton < 0 || auton >= (int)autons.size()) auton = 1;
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

    lift.tare_position();
    clawMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


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
            pros::lcd::print(6, "Lift: %.1f", lift.get_temperature());
            //pros::lcd::print(7, "PTO mode: %s", ptoMode() == PtoMode::COMBINED ? "22W" : "split");

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

// Scoring levels are generated from one constant spacing.
// IMPORTANT: Tune SCORE_SPACING to the measured motor degrees
// for one pin + cup stack.
constexpr double SCORE_SPACING = 400;
constexpr double SCORE_MIN_POS = 0;
constexpr double SCORE_MAX_POS = 1600;


void opcontrol() {
    chassis.setPose(0, 0, 0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    printf("\nDriver Control Started\n");
    opControl = true;
    bool movingToPickup = false;

    enum class ClawState {
        OFF,
        FORWARD,
        REVERSE
    };

    ClawState clawState = ClawState::OFF;
    uint32_t clawReverseStart = 0;

    while (true) {
        // Isolated cascade test: stop drivetrain, engage the PTO, then run both
        // cascade motors at full voltage. Wait safely while the PTO is switching.
        if (CASCADE_TEST_MODE) {
            chassis.arcade(0, 0);
            lift.move_voltage(12000);
            pros::delay(10);
            continue;
        }

        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // rajeev drive
        chassis.arcade(rightY, 0.95 * leftX);
        // shuhul drive
        // chassis.arcade(leftY, rightX);

          

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) toggler.toggle();

        bool bPressed = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B);
        bool r1Pressed = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1);
        bool r2Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool l1Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool l2Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);


        if (bPressed) movingToPickup = !movingToPickup;

        if (movingToPickup) {
            lift.move_absolute(CASCADE_PICKUP_POS, 150);
            if (fabs(lift.get_position() - CASCADE_PICKUP_POS) < 10) {
                movingToPickup = false;
            }
        } else{
            if (r2Held) lift.move(127);
            else if (l2Held) lift.move(-96);
            else lift.brake();
        }

        // ! CHANGE CLAW TO A TOGGLE BUTTON IF CONSTANT PRESSURE NEEDS TO BE APPLIED


        // R1 toggle:
        // First press  -> forward
        // Second press -> reverse for 0.5 seconds
        // Then         -> 0 voltage
        if (r1Pressed) {
            if (clawState == ClawState::OFF) {
                clawState = ClawState::FORWARD;
                clawMotor.move(127);
            }
            else if (clawState == ClawState::FORWARD) {
                clawState = ClawState::REVERSE;
                clawReverseStart = pros::millis();
                clawMotor.move(-127);
            }
        }

        if (clawState == ClawState::REVERSE) {
            if (pros::millis() - clawReverseStart >= 500) {
                clawMotor.move_voltage(0);
                clawState = ClawState::OFF;
            }
        }

        // L1 still manually runs the claw backward while held.
        // Don't let it interrupt the automatic 0.5-second reversal.
        if (l1Held && clawState != ClawState::REVERSE) {
            clawMotor.move(-127);
        }
        else if (clawState == ClawState::OFF && !l1Held) {
            clawMotor.move_voltage(0);
        }
        
        pros::delay(10);
    }
}   