#include "autons.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "globals.hpp"
#include <array> // IWYU pragma: keep
#include <cstdio>
#include <vector>
#include <tuple>
#include "lemlib/util.hpp"
#include "liblvgl/llemu.hpp" // IWYU pragma: keep
#include "pros/llemu.hpp" // IWYU pragma: keep
#include "pros/motors.h" // IWYU pragma: keep
#include "pros/rtos.hpp" // IWYU pragma: keep
#include <cstddef> // IWYU pragma: keep


// auton 
int auton = 9;
extern bool screenTaskRunning;


lemlib::Pose origin(0, 0, 0);

lemlib::Pose RightStandardStart(17.3, -48.6, 14);
lemlib::Pose LeftStandardStart(-17.3, -48.6, -14);
lemlib::Pose leftAWPStart(-17.7, -48.7, -90);
lemlib::Pose rightAWPStart(17, -48.7, 90);
lemlib::Pose SoloStart(-10, -45, 20);
lemlib::Pose SoloSigStart(7.4, -47.5, -90);



// vector of tuple of function description and pointers
std::vector<std::tuple<std::string, void(*)()>> autons = {

};

