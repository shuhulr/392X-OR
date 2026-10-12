#include "pto.hpp"
#include "globals.hpp"
#include "pros/rtos.hpp"
#include <cstdint>


constexpr int PTO_MOTOR_PORT = -17;                // negative means reversed; sign that runs the intake in
constexpr char PTO_PISTON_PORT = 'C';
constexpr bool PTO_COMBINED_WHEN_EXTENDED = true; // piston extended = both motors on the cascade
constexpr int PTO_FOLLOW_SIGN = 1;                // -1 if ptoMotor must spin opposite liftMotor to also push the cascade up
constexpr uint32_t PTO_SWITCH_MS = 250;           // time for the piston to actuate and the PTO to mesh

// !TODO tune these
// highest position the single cascade can hold
// the pto only switches back to SPLIT at or below this height, so it
// must be >= CASCADE_PICKUP_POS or Pickup can never switch back to SPLIT.
double PTO_SPLIT_MAX_POS = 1300;

pros::Motor ptoMotor(PTO_MOTOR_PORT, pros::MotorGearset::green);
// the bot powers on in the load pose, so the PTO starts in SPLIT
pros::adi::Pneumatics ptoPiston(PTO_PISTON_PORT, !PTO_COMBINED_WHEN_EXTENDED);

static PtoMode current = PtoMode::SPLIT; // engaged mode
static PtoMode wantedMode = PtoMode::SPLIT;
static PtoMode target = PtoMode::SPLIT;  // mode being switched to
static bool switching = false;
static uint32_t switchStart = 0;

static void setPiston(PtoMode m) {
    bool extend = (m == PtoMode::COMBINED) == PTO_COMBINED_WHEN_EXTENDED;
    if (extend) ptoPiston.extend();
    else ptoPiston.retract();
}

void ptoRequest(PtoMode wanted) { wantedMode = wanted; }
bool ptoReady() { return !switching; }
PtoMode ptoMode() { return current; }

void ptoUpdate() {
    if (switching) {
        ptoMotor.move(0); // stay still while the PTO meshes
        if (pros::millis() - switchStart >= PTO_SWITCH_MS) {
            current = target;
            switching = false;
        }
        return;
    }

    if (wantedMode != current) {
        // never drop to one motor while the cascade is too high for it to hold
        bool safe = wantedMode == PtoMode::COMBINED || liftMotor.get_position() <= PTO_SPLIT_MAX_POS;
        if (safe) {
            switching = true;
            target = wantedMode;
            switchStart = pros::millis();
            ptoMotor.move(0);
            setPiston(target);
            return;
        }
    }

    if (current == PtoMode::COMBINED) {
        // ptoMotor's own encoder is meaningless here, so mirror liftMotor's output instead
        ptoMotor.move_voltage(liftMotor.get_voltage() * PTO_FOLLOW_SIGN);
    }
}

void intake(int voltage) {
    if (current != PtoMode::SPLIT || switching) return;
    ptoMotor.move(voltage);
    intaking = true;
}

void stopIntake() {
    intaking = false;
    if (current != PtoMode::SPLIT || switching) return;
    ptoMotor.move(0);
}