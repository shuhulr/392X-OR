#pragma once
#include "pros/adi.hpp"
#include "pros/motors.hpp"


//   SPLIT:    liftMotor drives the cascade (11W), ptoMotor drives the intake (11W)
//   COMBINED: both motors drive the cascade (22W); the intake is unavailable
enum class PtoMode { SPLIT, COMBINED };

extern double PTO_SPLIT_MAX_POS;

extern pros::Motor ptoMotor;

// ptoRequest() with the mode you want then ptoUpdate().
// ptoUpdate() runs the switch (piston + settle time) and, in COMBINED, makes
// ptoMotor follow liftMotor.
void ptoRequest(PtoMode wanted);
void ptoUpdate();

bool ptoReady();    // false while the piston is mid-switch (don't move the cascade)
PtoMode ptoMode();  // the mode that is actually engaged right now

// intake commands; ignored unless the PTO is in SPLIT and not mid-switch
void intake(int voltage);
void stopIntake();