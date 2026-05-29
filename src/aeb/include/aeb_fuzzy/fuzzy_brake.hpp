#pragma once
#include <algorithm>

class FuzzyBrakeController
{
public:
  float compute_brake(float distance, float closing_speed, float /*ego_speed*/)
  {
    // EMERGENCY ZONE
    if (distance <= 6.0)
      return 1.0;   // EMERGENCY

    // CLOSE RANGE
    if (distance <= 12.0)
    {
      if (closing_speed > 5.0) return 0.65; // HARD
      if (closing_speed > 2.0) return 0.45; // MODERATE
      return 0.25;                          // LIGHT
    }

    // MEDIUM RANGE
    if (distance <= 25.0)
    {
      if (closing_speed > 5.0) return 0.45; // MODERATE
      return 0.20;                          // VERY LIGHT
    }

    // FAR
    return 0.0;
  }
};
