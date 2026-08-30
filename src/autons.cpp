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
int auton = 0;
extern bool screenTaskRunning;

lemlib::Pose origin(0, 0, 0);

// Starting poses for this season's autons go here once field setup is
// finalized, e.g.:
//   lemlib::Pose RightStart(x, y, heading);

// register each auton as {"Display Name", functionPointer}. Keep at least
// one entry so the brain screen / autonomous() never index out of bounds.
std::vector<std::tuple<std::string, void(*)()>> autons = {
    {"Nothing", []() {}},
};