#pragma once
#include <algorithm>
#include <cmath>

class FuzzyBrakeController
{
public:
  float compute_brake(float distance, float closing_speed, float ego_speed_kmh)
  {
    // Ignore objects that are completely out of range
    if (distance > 100.0f) return 0.0f;
    
    // Absolute critical emergency static distance (prevent creeping into stopped car)
    if (distance <= 6.0f) return 1.0f; // 100% Brake
    
    float ttc = 999.0f;
    if (closing_speed > 0.1f) {
      ttc = distance / closing_speed;
    }
    
    // 1. Time-To-Collision (TTC) Zone
    if (ttc <= 1.5f) return 1.0f;   // EMERGENCY
    if (ttc <= 2.5f) return 0.65f;  // HARD
    if (ttc <= 3.5f) return 0.40f;  // MODERATE
    
    // 2. Safe Following Distance (Headway) Zone
    // This is crucial for cut-ins where closing speed is near 0 
    // but the object is dangerously close.
    float ego_speed_ms = ego_speed_kmh / 3.6f;
    
    // Dynamic Safe Distance: 2 seconds headway + 6 meters static buffer
    float safe_dist = (ego_speed_ms * 2.0f) + 6.0f;
    
    // If the object is within the safe following distance, apply brakes
    // scaled by how deep into the safe zone they are.
    if (distance < safe_dist) {
      float danger_ratio = 1.0f - (distance / safe_dist); // 0.0 at edge, 1.0 at 0m distance
      
      if (danger_ratio > 0.6f) return 0.70f;
      if (danger_ratio > 0.3f) return 0.40f;
      return 0.20f; // Very Light / Preventative
    }
    
    // Default: Safe distance, no brake needed
    return 0.0f;
  }
};