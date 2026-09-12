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

extern int auton;
bool competitionInitialize = false;
bool opControl = false;

// screen task flag
bool screenTaskRunning = true;

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
// NOTE: ports 1 and 4 (the front motors) are 5.5W motors, while 2/3/5/6 are
// 11W. Mixing wattages in one group is a known trouble spot - LemLib issue
// #186 documents 5.5W motors not always being ratioed the same as 11W ones
// in a group. Keep an eye on drift/uneven power between the front and rear
// motors; if you see it, that's likely why.
pros::MotorGroup leftMotors({-1, -2, -3}, pros::MotorGearset::blue);  // left motor group
pros::MotorGroup rightMotors({4, 5, 6}, pros::MotorGearset::blue);   // right motor group

// intake (port 7, reversed)
pros::Motor intakeMotor(-7);

// lift - cascade (port 12)
pros::Motor liftMotor(12);

// two-bar / Lady Brown style mechanism (ports 15, 16 reversed, both 5.5W)
pros::MotorGroup twoBar({15, -16});

// two-bar preset positions - TODO: tune once the mechanism is built out
const double TWO_BAR_STOW_POS = 0;
const double TWO_BAR_SCORE_POS = 90;

// game color (0 for red, 1 for blue, -1 for none)
int gameColor = -1;

// pneumatics
pros::adi::Pneumatics matchloader('H', false);

// Inertial Sensor (port 11)
pros::Imu imu(11);

// tracking wheels
// vertical odometry rotation sensor, port 9, reversed
pros::Rotation verticalEnc(-9);
// horizontal odometry rotation sensor, port 10, reversed
pros::Rotation horizontalEnc(-10);
// horizontal tracking wheel. 2" diameter, 3.7" offset behind center (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, 2, -3.7);
// vertical tracking wheel. 2" diameter, 0.4" offset left of center (negative)
lemlib::TrackingWheel vertical(&verticalEnc, 2, -0.4);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              11.5,                        // track width (in)
                              lemlib::Omniwheel::NEW_325,  // 3.25" omnis
                              450,                          // drivetrain rpm
                              2                             // horizontal drift
);

// lateral motion controller
lemlib::ControllerSettings linearController(5.5,  // kP
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

// secondary angular controller, used for tighter U30-class turns
lemlib::ControllerSettings angularControllerU30(4.3,
                                             0.28,
                                             20,
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
lemlib::Chassis chassis(drivetrain, linearController, angularController, angularControllerU30, sensors, &throttleCurve, &steerCurve);

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
    chassis.setPose(-48, -48, 0);

#if RCL_ENABLED
    RclMain.setRclPose(chassis.getPose());
    RclMain.updateBotPose(&backRcl);
    RclMain.updateBotPose(&leftRcl);
#endif

    // hold position when no voltage is applied, instead of coasting/falling
    liftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // zero the two-bar's position so TWO_BAR_STOW_POS/SCORE_POS are relative
    // to wherever it's built to rest at startup
    twoBar.tare_position();

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
            pros::lcd::print(6, "Intake: %.1f  Lift: %.1f", intakeMotor.get_temperature(), liftMotor.get_temperature());

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

/**
 * Runs in driver control
 */
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

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
            intake(127);
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
            intake(-128);
        // TODO: make this one time check flag
        else
            stopIntake();

        // lift: hold B to raise, hold down to lower
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            liftMotor.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            liftMotor.move(-127);
        } else {
            liftMotor.move(0);
        }

        // two-bar: while L2 is held, go to scoring position; otherwise return to stow
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            twoBar.move_absolute(TWO_BAR_SCORE_POS, 100);
        } else {
            twoBar.move_absolute(TWO_BAR_STOW_POS, 100);
        }

        pros::delay(10);
    }
}