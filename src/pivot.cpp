#include "pivot.hpp"

#include "globals.hpp"
#include "pto.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace {
// Negative port reverses the sensor direction so pivot-up reads as increasing.
pros::Rotation pivotRotation(19);

// Pivot feedback controller. The motor and sensor rotate in opposite reported
// directions on this robot, so the voltage sign is inverted below.
constexpr double PIVOT_KP_MV_PER_DEG = 110.0;
constexpr double PIVOT_TOLERANCE_DEG = 5.0;
constexpr double PIVOT_MIN_MV = 1200.0;
constexpr double PIVOT_MAX_MV = 12000.0;

// Transition tuning: the cascade gets a short head start, then keeps rising
// slowly while the pivot swings out. It completes on measured pivot position,
// not just elapsed time, and has a timeout as a fallback.
constexpr uint32_t PIVOT_LIFT_HEADSTART_MS = 650;
constexpr uint32_t PIVOT_RAMP_MS = 1000;
constexpr uint32_t TRANSITION_TIMEOUT_MS = 2200;
constexpr int TRANSITION_LIFT_SPEED = 65; // PROS move() range: -127..127
constexpr int TRANSITION_PIVOT_MAX_PERCENT = 70;
constexpr double TRANSITION_COMPLETE_TOLERANCE_DEG = 3.0;

bool transitionActive = false;
uint32_t transitionStartMs = 0;
uint32_t lastUpdateMs = 0;
double transitionStartPivot = 0.0;

double pivotTarget = PIVOT_DOWN_POS;
int pivotTargetMaxPercent = 100;

// Ease-in/ease-out-ish trajectory used for the pivot target.
double trajectoryProgress(double t) {
    t = std::clamp(t, 0.0, 1.0);
    return 2.0 * t * t - t * t * t;
}

void transitionLiftSlowly() {
    if (ptoReady()) {
        // Use open-loop velocity during the transition. A continuously creeping
        // move_absolute target can stay just ahead of the motor and produce
        // little real lift movement. Regular scoring takes over after pivot-out.
        liftMotor.move(TRANSITION_LIFT_SPEED);
    } else {
        liftMotor.brake();
    }
}
} // namespace

void pivotInitialize() {
    
    pivotMotor.set_encoder_units(pros::motor_encoder_units_e::E_MOTOR_ENCODER_DEGREES);
    pivotRotation.set_position(4500);
    pivotMotor.set_zero_position(45); // LOAD pose is the 5-degree reference (centidegrees)
    transitionActive = false;
}

double pivotPositionDegrees() {
    // PROS Rotation::get_position() returns centidegrees.
    return pivotRotation.get_position() / 100.0;
    
    // return pivotMotor.get_position() / (-3);
}

void pivotTo(double targetDegrees, int maxPercent) {
    pivotTarget = targetDegrees;
    pivotTargetMaxPercent = std::clamp(maxPercent, 0, 100);
    applyPivotTarget();
}

void pivotUpdateControl() {
    applyPivotTarget();
}
   
void applyPivotTarget() {
    const double error = pivotTarget - pivotPositionDegrees();

    if (std::fabs(error) <= PIVOT_TOLERANCE_DEG) {
        pivotMotor.move_voltage(0);
        return;
    }

    double voltage = -error * PIVOT_KP_MV_PER_DEG;

    const double maxVoltage =
        PIVOT_MAX_MV * pivotTargetMaxPercent / 100.0;

    voltage = std::clamp(voltage, -maxVoltage, maxVoltage);

    if (std::fabs(voltage) < PIVOT_MIN_MV) {
        voltage = std::copysign(
            std::min(PIVOT_MIN_MV, maxVoltage), voltage
        );
    }

    pivotMotor.move_voltage(static_cast<std::int32_t>(voltage));
    // printf("Target: %.2f | Position: %.2f | Error: %.2f, Voltage: %lf\n",pivotTarget,pivotPositionDegrees(),pivotTarget - pivotPositionDegrees(),voltage);
}

bool pivotBeginScoringTransition() {
    transitionStartPivot = pivotPositionDegrees();
    transitionStartMs = pros::millis();
    lastUpdateMs = transitionStartMs;

    transitionActive = std::fabs(PIVOT_OUT_POS - transitionStartPivot) > TRANSITION_COMPLETE_TOLERANCE_DEG;
    return transitionActive;
}

PivotTransitionStatus pivotUpdateScoringTransition() {
    if (!transitionActive) return PivotTransitionStatus::IDLE; 

    const uint32_t now = pros::millis();
    const uint32_t dt = now - lastUpdateMs;
    lastUpdateMs = now;

    // Pause the transition clock during PTO movement. Hold the pivot at its
    // current angle and do not command the cascade while the PTO is disengaged.
    if (!ptoReady()) {
        transitionStartMs += dt;
        liftMotor.brake();
        pivotTo(pivotPositionDegrees(), TRANSITION_PIVOT_MAX_PERCENT);
        return PivotTransitionStatus::RUNNING;
    }

    const uint32_t elapsed = now - transitionStartMs;
    if (elapsed >= TRANSITION_TIMEOUT_MS) {
        transitionActive = false;
        return PivotTransitionStatus::TIMED_OUT;
    }

    // Keep the cascade moving at a steady, deliberately slow speed throughout
    // the transition. Do not creep a position target forward: that can leave
    // the target only a few degrees ahead of the motor and barely move the lift.
    transitionLiftSlowly();

    // Let the cascade move first, then start the pivot while the cascade target
    // continues to rise. Keep commanding the final target until the sensor says
    // the pivot is physically out, or the timeout expires.
    if (elapsed < PIVOT_LIFT_HEADSTART_MS) {
        pivotTo(transitionStartPivot, TRANSITION_PIVOT_MAX_PERCENT);
    } else {
        const double pivotT = static_cast<double>(elapsed - PIVOT_LIFT_HEADSTART_MS) /
                              static_cast<double>(PIVOT_RAMP_MS);
        const double u = trajectoryProgress(pivotT);
        const double pivotTarget = transitionStartPivot + (PIVOT_OUT_POS - transitionStartPivot) * u;
        pivotTo(pivotTarget, TRANSITION_PIVOT_MAX_PERCENT);
    }

    if (std::fabs(PIVOT_OUT_POS/2 - pivotPositionDegrees()) <= TRANSITION_COMPLETE_TOLERANCE_DEG) {
        transitionActive = false;
        return PivotTransitionStatus::COMPLETE;
    }

    return PivotTransitionStatus::RUNNING;
}

void pivotCancelScoringTransition() {
    transitionActive = false;
}

bool pivotMoveAbsolute(double targetDegrees,
                       int maxPercent,
                       uint32_t timeoutMs) {
    const uint32_t start = pros::millis();

    while (pros::millis() - start < timeoutMs) {
        const double position = pivotPositionDegrees();
        const double error = targetDegrees - position;

        if (std::fabs(error) <= PIVOT_TOLERANCE_DEG) {
            pivotMotor.move_voltage(0);
            return true;
        }

        pivotTo(targetDegrees, maxPercent);
        pros::delay(10);
    }

    // Stop actively driving if the move times out.
    pivotMotor.move_voltage(0);
    return false;
}