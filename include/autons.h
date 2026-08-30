#pragma once
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <vector>
#include <tuple>
#include <string>
 
// list of all registered autonomous routines: {display name, function pointer}
extern std::vector<std::tuple<std::string, void(*)()>> autons;

/*extern void PIDTest();
extern void BlueRightBonus();
extern void BlueRightBonus9ball();
extern void BlueLeftBonus();
extern void BlueLeftAWP();
extern void soloAWP();
extern void autonSkills();
extern void BlueRightAWP();
extern void BlueLeft9Ball2Goal();
extern void Left4Plus5();
extern void pidTest();
extern void lowGoalSkills();*/