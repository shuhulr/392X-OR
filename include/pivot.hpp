#pragma once

#include <cstdint>

// The rotation sensor is zeroed in the robot's LOAD pose during initialize().
constexpr double PIVOT_DOWN_POS = 0.0;
constexpr double PIVOT_OUT_POS = 95.0;

enum class PivotTransitionStatus {
    IDLE,
    RUNNING,
    COMPLETE,
    TIMED_OUT
};

// Call once from initialize() while the pivot is physically in the LOAD pose.
void pivotInitialize();

double pivotPositionDegrees();
void pivotTo(double targetDegrees, int maxPercent);

// Starts the coordinated R2 transition. Returns true if a transition is needed.
bool pivotBeginScoringTransition();

// Commands the cascade and pivot together while the transition is running.
// COMPLETE means the pivot reached its out target; TIMED_OUT means the fallback
// scoring behavior should take over.
PivotTransitionStatus pivotUpdateScoringTransition();
void pivotCancelScoringTransition();
