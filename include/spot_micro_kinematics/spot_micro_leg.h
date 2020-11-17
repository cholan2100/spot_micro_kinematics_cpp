#pragma once

#include "spot_micro_kinematics/utils.h"

#ifndef ESP32_DSP // uses array implementation with DSP
#include <eigen3/Eigen/Geometry>
#endif // ESP32_DSP

namespace smk {

class SpotMicroLeg {

 public:
  // Constructor, sets a leg up with initial joint angles and link lengths
  SpotMicroLeg(const JointAngles& joint_angles,
               const LinkLengths& link_lengths,
               bool is_leg_12);
 
  // Also use default constructor
  SpotMicroLeg() = default;

  // Sets the three leg angles and updates homogeneous transformation
  // matrices 
  void setAngles(const JointAngles& joint_angles);

  // Set the foot position in the leg's local coordinate system
  void setFootPosLocalCoordinates(const Point& point);

#ifndef ESP32_DSP // uses array implementation with DSP
  // Set the foot position in global coordinate system given a desired point, at
  // the ht representing the start of the leg's coordinate system
  void setFootPosGlobalCoordinates(const Point& point, 
                                   const Eigen::Matrix4f& ht_leg_start);

  // Returns the foot position in the global coordinate system given the ht
  // representing the start of the leg's coordinate system
  Point getFootPosGlobalCoordinates(const Eigen::Matrix4f& ht_leg_start);
#endif // ESP32_DSP

  // Returns the three joint angles, ang1, ang2, ang3
  JointAngles getLegJointAngles();

#ifdef ARRAY_IMPLEMENTATION
  void array_setFootPosGlobalCoordinates(const Point& point, 
                                   const float ht_leg_start[][4]);
  Point array_getFootPosGlobalCoordinates(const float ht_leg_start[][4]);
#endif //ARRAY_IMPLEMENTATION

 private:
  JointAngles joint_angles_; // Joint angles of the leg

  LinkLengths link_lengths_; // Lengths of the leg links

  bool is_leg_12_; // Boolean representing whether leg is 1 or 2,
                   // (as opposed to 3 or 4)
};


}
