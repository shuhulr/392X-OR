#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/logger/telemetrySink.hpp" // IWYU pragma: keep
#include "RclTracking.hpp"

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
extern bool ovrde;
bool competitionInitialize = false;
bool opControl = false;

// odom lift flag
//bool odomLiftRaise = false;

// screen task flag
bool screenTaskRunning = true;





// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-13, 14, -15}, pros::MotorGearset::blue); // left motor group - ports 1, 2 (reversed), 3
pros::MotorGroup rightMotors({10, 6, -8}, pros::MotorGearset::blue); // right motor group - ports 4 (reversed), 5, 6 (reversed)


// motors
pros::Motor intakeRight(19);
pros::Motor intakeLeft(-21);
pros::MotorGroup intakeM({19, -21});
// optical disconnect on port 12
// pros::Optical opticalSensor(12);


// game color (0 for red, 1 for blue, -1 for none)
int gameColor = -1;

// pneumatics
pros::adi::Pneumatics matchloader('H', false);


// Inertial Sensor on port 10
pros::Imu imu(20);


// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 8, not reversed
pros::Rotation horizontalEnc(12);
// vertical tracking wheel encoder. Rotation sensor, port 7, not reversed
pros::Rotation verticalEnc(-2);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, 2, -3.7);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, 2, -0.4);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(5.5, // proportional gain (kP)
                                            0.24, // integral gain (kI)
                                            12, // derivative gain (kD)
                                            2, // anti windup
                                            0.5, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            2, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            15 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(angular_kp, // proportional gain (kP)
                                             angular_ki, // integral gain (kI)
                                             angular_kd, // derivative gain (kD)
                                             5, // anti windup
                                             1, // small error range, in degrees
                                             75, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             250, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

lemlib::ControllerSettings angularControllerU30(4.3, // proportional gain (kP)
                                             0.28, // integral gain (kI)
                                             20, // derivative gain (kD)
                                             4, // anti windup
                                             1, // small error range, in degrees
                                             75, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             250, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
/*lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);*/

//sensors for rcl
lemlib::OdomSensors sensors(nullptr,
                            nullptr,
                            nullptr,
                            nullptr,
                            &imu
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(13, // joystick deadband out of 127
                                     13, // minimum output where drivetrain will move out of 127
                                     1 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(13, // joystick deadband out of 127
                                  13, // minimum output where drivetrain will move out of 127
                                  1 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, angularControllerU30, sensors, &throttleCurve, &steerCurve);


// distance sensors
pros::Distance leftDist(18);
pros::Distance rightDist(5);
pros::Distance backDist(17);

//rcl setup
inline RclSensor rightRcl(&rightDist, 3.1, 5.1, 90);
inline RclSensor leftRcl(&leftDist, -3.1, 5.1, 270);
inline RclSensor backRcl(&backDist, -3.25, -7, 180);
inline RclTracking RclMain(&chassis);

// loaders
inline Circle_Obstacle redUpLoader(-67.5, 46.5, 3);
inline Circle_Obstacle redDownLoader(-67.5, -46.5, 3);
inline Circle_Obstacle blueUpLoader(67.5, 46.5, 3);
inline Circle_Obstacle blueDownLoader(67.5, -46.5, 3);

// legs
inline Circle_Obstacle upLongGoalLeft(-21, 47.5, 4);
inline Circle_Obstacle upLongGoalRight(21, 47.5, 4);
inline Circle_Obstacle downLongGoalLeft(-21, -47.5, 4);
inline Circle_Obstacle downLongGoalRight(21, -47.5, 4);
inline Circle_Obstacle centerGoals(0, 0, 5);

// Disable Line for the autonomous period
inline Line_Obstacle disableLine(0, FIELD_NEG_HALF_LENGTH, 0, FIELD_HALF_LENGTH);


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

//using modulo (math)
void leftScreenButton() {
    auton = (auton - 1 + autons.size()) % (autons.size());
    updateAutoFile();
}

void rightScreenButton() {
    auton = (auton + 1) % (autons.size());
    updateAutoFile();
}


void displayImage();
void initialize() {
    //shotgunRS.set_position(0);
    
    RclMain.startTracking();
    
    FILE* file = fopen("/usd/auto.txt", "r");
    if (file != NULL) {
        char buf[3];        //two for digit, one for null terminator
        fread(buf, 1, 2, file);
        buf[2] = '\0';      // tells stoi where string ends
        
        auton = std::stoi(buf); 
        printf("Loaded Auton: %s\n", buf);
        fclose(file);
    } else {
        auton = 0; // default if no file is found
        printf("SD Card not found, defaulting to auton 0\n");
    }


    displayImage();
    
    
    chassis.calibrate(); // calibrate sensors


    chassis.setPose(-48, -48, 0); // or whatever your actual starting coords are
    RclMain.setRclPose(chassis.getPose());
    RclMain.updateBotPose(&backRcl);
    RclMain.updateBotPose(&leftRcl);


    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging


    //pros::Task pidTuner()

    pros::Task screenTask([&]() {
        while (screenTaskRunning) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %.4f", RclMain.getRclPose().x); // x
            pros::lcd::print(1, "Y: %.4f", RclMain.getRclPose().y); // y
            pros::lcd::print(2, "Theta: %.3f", chassis.getPose().theta); // heading
            
            pros::lcd::print(3, "auton index: %d", auton);
            pros::lcd::print(4, "%s", std::get<0>(autons[auton]));

            // auto leftResult = leftRcl.getBotCoord(chassis.getPose());
            // auto rightResult = backRcl.getBotCoord(chassis.getPose());
            //auto rightResult = rightRcl.getBotCoord(chassis.getPose());

            //pros::lcd::print(4, "R: type=%d val=%.2f", (int)rightResult.first, rightResult.second);

            
            pros::lcd::print(5, "Left: %f    Right: %f", leftMotors.get_temperature(), rightMotors.get_temperature());
            pros::lcd::print(6, "Intake: %f", intakeM.get_temperature());
            

            // pros::lcd::print(5, "L dist: %d  conf: %d", leftDist.get(), leftDist.get_confidence());
            // pros::lcd::print(6, "R dist: %d  conf: %d", rightDist.get(), rightDist.get_confidence());
            // pros::lcd::print(7, "B dist: %d  conf: %d", backDist.get(), backDist.get_confidence());
            // pros::lcd::print(3, "L: type=%d val=%.2f", (int)leftResult.first, leftResult.second);
            // pros::lcd::print(4, "B: type=%d val=%.2f", (int)rightResult.first, rightResult.second);
            // pros::lcd::print(5, "RCL: %.2f %.2f", RclMain.getRclPose().x, RclMain.getRclPose().y);
            // pros::lcd::print(6, "LEM: %.2f %.2f", chassis.getPose().x, chassis.getPose().y);
            // pros::lcd::print(7, "Ldiff: %.2f", std::abs(leftResult.second - RclMain.getRclPose().x));

            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // telemetry.log(Level level, fmt::format_string<T...> format, T &&args...)
            // delay to save resources
            pros::delay(50);
        }
    });
    
    
}


/**
 * Runs while the robot is disabled
 */
void disabled() {

    
}

/**
 * runs after initialize if the robot is connected to field control
 */



void competition_initialize() {
    competitionInitialize = true;
}

// get a path used for pure pursuit
// this needs to be put outside a function
//ASSET(example_txt); // '.' replaced with "_" to make c++ happy


void autonomous() {
    // screenTaskRunning = false; // stop the screen task during auton

    
    //autonIndex = 0; // Change this to whichever auton you want to run
    std::get<1>(autons[auton])();
    
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    printf("\nDriver Control Started\n");
    opControl = true;

    while (true) {
        // get joystick positions
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        // move the chassis with curvature drive
        //rajeev drive
        chassis.arcade(rightY, 0.95 * leftX);

        //shuhul drive
        //chassis.arcade(leftY, rightX);

        

        pros::delay(10);
    }
}