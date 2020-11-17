#include <eigen3/Eigen/Geometry>
#include <iostream>

#include "spot_micro_kinematics/utils.h"
#include "spot_micro_kinematics/spot_micro_leg.h"
#include "spot_micro_kinematics/spot_micro_kinematics.h"

using namespace Eigen;

namespace smk {

SpotMicroKinematics::SpotMicroKinematics(float x, float y, float z,
                                         const SpotMicroConfig& smc) 
    : x_(x),
      y_(y),
      z_(z),
      smc_(smc) {

  // Initialize other class attributes
  phi_ = 0.0f;
  theta_ = 0.0f;
  psi_ = 0.0f;
  
  // Create temporary structs for initializing leg's joint angles and lengths  
  JointAngles joint_angles_temp = {0.0f, 0.0f, 0.0f};
  LinkLengths link_lengths_temp = {smc.hip_link_length,
                                   smc.upper_leg_link_length,
                                   smc.lower_leg_link_length}; 

  // Create legs
  right_back_leg_  = SpotMicroLeg(joint_angles_temp, link_lengths_temp, true);
  right_front_leg_ = SpotMicroLeg(joint_angles_temp, link_lengths_temp, true);
  left_front_leg_  = SpotMicroLeg(joint_angles_temp, link_lengths_temp, false);
  left_back_leg_   = SpotMicroLeg(joint_angles_temp, link_lengths_temp, false);
}

#define ARRAY_IMPLEMENTATION
#ifdef ARRAY_IMPLEMENTATION
typedef Matrix<float,4,4,RowMajor> Matrix4fRM;
void SpotMicroKinematics::array_getBodyHt(float ht[][4]) {
  float Txyz[4][4] = {0,}; 
  array_homogTransXyz(x_, y_, z_, Txyz);
  float Rxyz[4][4] = {0,};
  array_homogRotXyz(phi_, psi_, theta_, Rxyz);
  dsps_add_f32_ae32((float*) Txyz, (float*) Rxyz, (float*) ht, 16, 1, 1, 1);
}
#endif //ARRAY_IMPLEMENTATION

Matrix4f SpotMicroKinematics::getBodyHt() {
  // Euler angle order is phi, psi, theta because the axes of the robot are x
  // pointing forward, y pointing up, z pointing right
#ifdef ARRAY_IMPLEMENTATION
  float ht[4][4] = {0,};
  array_getBodyHt(ht);
  return Map<Matrix4fRM>(&ht[0][0], 4, 4);
#else // ARRAY_IMPLEMENTATION
  return(homogTransXyz(x_, y_, z_) * homogRotXyz(phi_, psi_, theta_));
#endif // ARRAY_IMPLEMENTATION
}


void SpotMicroKinematics::setLegJointAngles(
    const LegsJointAngles& four_legs_joint_angs) {
  // Call each leg's method to set joint angles 
  right_back_leg_.setAngles(four_legs_joint_angs.right_back);
  right_front_leg_.setAngles(four_legs_joint_angs.right_front);
  left_front_leg_.setAngles(four_legs_joint_angs.left_front);
  left_back_leg_.setAngles(four_legs_joint_angs.left_back);
}

void SpotMicroKinematics::setFeetPosGlobalCoordinates(
    const LegsFootPos& four_legs_foot_pos) {
  // Get the body center homogeneous transform matrix 
#ifdef ARRAY_IMPLEMENTATION
  float array_ht_body[4][4];
  array_getBodyHt(array_ht_body);
#else // ARRAY_IMPLEMENTATION
  Matrix4f ht_body = getBodyHt();
#endif // ARRAY_IMPLEMENTATION
  
  // Create each leg's starting ht matrix. Made in order of right back, right 
  // front, left front, left back
#ifdef ARRAY_IMPLEMENTATION
  float array_ht_rb[4][4] = {0,};
  array_htLegRightBack(array_ht_body,smc_.body_length, smc_.body_width, array_ht_rb);
  float array_ht_rf[4][4] = {0,};
  array_htLegRightFront(array_ht_body,smc_.body_length, smc_.body_width, array_ht_rf);
  float array_ht_lf[4][4] = {0,};
  array_htLegLeftFront(array_ht_body,smc_.body_length, smc_.body_width, array_ht_lf);
  float array_ht_lb[4][4] = {0,};
  array_htLegLeftBack(array_ht_body,smc_.body_length, smc_.body_width, array_ht_lb);
#else // ARRAY_IMPLEMENTATION
  Matrix4f ht_rb = htLegRightBack(ht_body, smc_.body_length, smc_.body_width);
  Matrix4f ht_rf = htLegRightFront(ht_body, smc_.body_length, smc_.body_width);
  Matrix4f ht_lf = htLegLeftFront(ht_body, smc_.body_length, smc_.body_width);
  Matrix4f ht_lb = htLegLeftBack(ht_body, smc_.body_length, smc_.body_width);
#endif // ARRAY_IMPLEMENTATION

  // Call each leg's method to set foot position in global coordinates
#ifdef ARRAY_IMPLEMENTATION
  right_back_leg_.array_setFootPosGlobalCoordinates(
      four_legs_foot_pos.right_back, array_ht_rb);

  right_front_leg_.array_setFootPosGlobalCoordinates(
      four_legs_foot_pos.right_front, array_ht_rf);

  left_front_leg_.array_setFootPosGlobalCoordinates(
      four_legs_foot_pos.left_front, array_ht_lf);

  left_back_leg_.array_setFootPosGlobalCoordinates(
      four_legs_foot_pos.left_back, array_ht_lb);
#else // ARRAY_IMPLEMENTATION
  right_back_leg_.setFootPosGlobalCoordinates(
      four_legs_foot_pos.right_back, ht_rb);

  right_front_leg_.setFootPosGlobalCoordinates(
      four_legs_foot_pos.right_front, ht_rf);

  left_front_leg_.setFootPosGlobalCoordinates(
      four_legs_foot_pos.left_front, ht_lf);

  left_back_leg_.setFootPosGlobalCoordinates(
      four_legs_foot_pos.left_back, ht_lb);
#endif // ARRAY_IMPLEMENTATION
}


void SpotMicroKinematics::setBodyAngles(float phi, float theta, float psi) {
  // Save the current feet position
  LegsFootPos saved_foot_pos = getLegsFootPos();

  // Update body angles
  phi_ = phi;
  theta_ = theta;
  psi_ = psi;

  // Call method to set absolute feet position to the saved values
  setFeetPosGlobalCoordinates(saved_foot_pos);
}


void SpotMicroKinematics::setBodyPosition(float x, float y, float z) {
  // Save the current feet position
  LegsFootPos saved_foot_pos = getLegsFootPos();

  // Update body angles
  x_ = x;
  y_ = y;
  z_ = z;

  // Call method to set absolute feet position to the saved values
  setFeetPosGlobalCoordinates(saved_foot_pos);
}


void SpotMicroKinematics::setBodyState(const BodyState& body_state) {
  // Set x,y,z position
  x_ = body_state.xyz_pos.x; 
  y_ = body_state.xyz_pos.y; 
  z_ = body_state.xyz_pos.z;

  // set euler angles
  phi_ = body_state.euler_angs.phi;
  theta_ = body_state.euler_angs.theta;
  psi_ = body_state.euler_angs.psi;

  // set feet position
  setFeetPosGlobalCoordinates(body_state.leg_feet_pos);
}

LegsJointAngles SpotMicroKinematics::getLegsJointAngles() {
  // Return the leg joint angles
  LegsJointAngles ret_val;

  ret_val.right_back = right_back_leg_.getLegJointAngles();
  ret_val.right_front = right_front_leg_.getLegJointAngles();
  ret_val.left_front = left_front_leg_.getLegJointAngles();
  ret_val.left_back = left_back_leg_.getLegJointAngles();

  return ret_val;
}


LegsFootPos SpotMicroKinematics::getLegsFootPos() {
  
  // Get the body center homogeneous transform matrix 
#ifndef ARRAY_IMPLEMENTATION
//TODO: implement a cached version for optimization
  Matrix4f ht_body = getBodyHt();
#else // ARRAY_IMPLEMENTATION
  float array_ht_body[4][4] = {0,}; // Transformation Matrix
  array_getBodyHt(array_ht_body);
#endif //ARRAY_IMPLEMENTATION

  // Return the leg joint angles
  LegsFootPos ret_val;

  // Create each leg's starting ht matrix. Made in order of right back, right 
  // front, left front, left back
#ifndef ARRAY_IMPLEMENTATION
  Matrix4f ht_rb = htLegRightBack(ht_body, smc_.body_length, smc_.body_width);
  Matrix4f ht_rf = htLegRightFront(ht_body, smc_.body_length, smc_.body_width);
  Matrix4f ht_lf = htLegLeftFront(ht_body, smc_.body_length, smc_.body_width);
  Matrix4f ht_lb = htLegLeftBack(ht_body, smc_.body_length, smc_.body_width);

  ret_val.right_back  = right_back_leg_.getFootPosGlobalCoordinates(ht_rb);
  ret_val.right_front = right_front_leg_.getFootPosGlobalCoordinates(ht_rf);
  ret_val.left_front  = left_front_leg_.getFootPosGlobalCoordinates(ht_lf);
  ret_val.left_back   = left_back_leg_.getFootPosGlobalCoordinates(ht_lb);
#else // ARRAY_IMPLEMENTATION
  float array_ht_rb[4][4] = {0,};
  array_htLegRightBack(array_ht_body,smc_.body_length, smc_.body_width, array_ht_rb);
  float array_ht_rf[4][4] = {0,};
  array_htLegRightFront(array_ht_body,smc_.body_length, smc_.body_width, array_ht_rf);
  float array_ht_lf[4][4] = {0,};
  array_htLegLeftFront(array_ht_body,smc_.body_length, smc_.body_width, array_ht_lf);
  float array_ht_lb[4][4] = {0,};
  array_htLegLeftBack(array_ht_body,smc_.body_length, smc_.body_width, array_ht_lb);

  ret_val.right_back  = right_back_leg_.array_getFootPosGlobalCoordinates(array_ht_rb);
  ret_val.right_front = right_front_leg_.array_getFootPosGlobalCoordinates(array_ht_rf);
  ret_val.left_front  = left_front_leg_.array_getFootPosGlobalCoordinates(array_ht_lf);
  ret_val.left_back   = left_back_leg_.array_getFootPosGlobalCoordinates(array_ht_lb);
#endif //ARRAY_IMPLEMENTATION

  return ret_val;
}

BodyState SpotMicroKinematics::getBodyState() {
  BodyState body_state = {EulerAngs{phi_, theta_, psi_},
                          Point{x_, y_, z_},
                          getLegsFootPos()};
  return body_state;
}

}
