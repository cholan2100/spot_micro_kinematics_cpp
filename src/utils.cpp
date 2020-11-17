#include "spot_micro_kinematics/utils.h"

#include <iostream>
#include <cmath>

#include <eigen3/Eigen/Geometry>

using namespace Eigen;

namespace smk
{

typedef Matrix<float,4,4,RowMajor> Matrix4fRM;

Matrix4f homogRotXyz(float x_ang, float y_ang, float z_ang)
{
  // Create 3d transformation, and execute x, y, and z rotations
#ifndef ARRAY_IMPLEMENTATION
  Transform<float, 3, Affine> t = Transform<float,3,Affine>::Identity();
  t.rotate(AngleAxisf(x_ang, Vector3f::UnitX()));
  t.rotate(AngleAxisf(y_ang, Vector3f::UnitY()));
  t.rotate(AngleAxisf(z_ang, Vector3f::UnitZ()));

  return t.matrix();
#else //ARRAY_IMPLEMENTATION
  float Rxyz[4][4] = {0,};
  array_homogRotXyz(x_ang, y_ang, z_ang, Rxyz);
  return Map<Matrix4fRM>(&Rxyz[0][0], 4, 4);
#endif //ARRAY_IMPLEMENTATION
}


Matrix4f homogTransXyz(float x, float y, float z)
{
  // Create a linear translation homogenous transformation matrix
#ifndef ARRAY_IMPLEMENTATION
  Transform<float, 3, Affine> t;
  t = Translation<float, 3> (Vector3f(x,y,z));

  return t.matrix();
#else //ARRAY_IMPLEMENTATION
  float Txyz[4][4] = {0,};
  Txyz[0][0] = 1; Txyz[1][1] = 1; Txyz[2][2] = 1; Txyz[3][3] = 1;
  array_homogTransXyz(x, y, z, Txyz);
  return Map<Matrix4fRM>(&Txyz[0][0], 4, 4);
#endif //ARRAY_IMPLEMENTATION
}

Matrix4f homogInverse(const Matrix4f& ht)
{
#ifndef ARRAY_IMPLEMENTATION
//The inverse of a homogeneous transformation matrix can be represented as a
//    a matrix product of the following:
//
//                -------------------   ------------------- 
//                |           |  0  |   | 1   0   0  -x_t |
//    ht_inv   =  |   R^-1    |  0  | * | 0   1   0  -y_t |
//                |___________|  0  |   | 0   0   1  -z_t |
//                | 0   0   0 |  1  |   | 0   0   0   1   |
//                -------------------   -------------------
//
//    Where R^-1 is the inverse of the rotation matrix portion of the homogeneous
//    transform (the first three rows and columns). Note that the inverse
//    of a rotation matrix is equal to its transpose. And x_t, y_t, z_t are the
//    linear trasnformation portions of the original transform.  

  Matrix3f temp_rot = ht.block<3,3>(0,0); // Get rotation matrix portion from homogeneous transform via block
  temp_rot.transposeInPlace();    // Transpose, equivalent to inverse for rotation matrix

  // Store linear translation portion and negate directions
  Vector3f temp_translate = ht.block<3,1>(0,3);
  temp_translate = temp_translate * -1.0f;

  // Create left hand portion of ht_inv from comment block above
  Matrix4f ht_inverted1 = Matrix4f::Identity();
  ht_inverted1.block<3,3>(0,0) = temp_rot;

  // Create right hand portion of ht_in from comment block above
  Matrix4f ht_inverted2 = Matrix4f::Identity();
  ht_inverted2.block<3,1>(0,3) = temp_translate;

  // Return product of matrices, the homogeneous transform inverse
  return (ht_inverted1 * ht_inverted2);
#else // ARRAY_IMPLEMENTATION
  float array_ht[4][4];
  Map<Matrix4fRM>(&array_ht[0][0], 4, 4) = ht;
  float inv[4][4];
  array_homogInverse(array_ht, inv);
  return Map<Matrix4fRM>(&inv[0][0], 4, 4);
#endif // ARRAY_IMPLEMENTATION
}

Matrix4f htLegRightBack(const Matrix4f& ht_body_center, float body_length, float body_width) {
#ifndef ARRAY_IMPLEMENTATION
  // Build up matrix representing right back leg ht. First, a pi/2 rotation in y
  Matrix4f htLegRightBack = homogRotXyz(0.0f, M_PI/2.0f, 0.0);

  // Next, add the linear translation portion
  htLegRightBack.block<3,1>(0,3) = Vector3f(-body_length/2.0f, 0.0f, body_width/2.0f);

  // Return the matrix product of ht_body_center and the leg rightback ht
  return (ht_body_center * htLegRightBack);
#else // ARRAY_IMPLEMENTATION
  // implement using arrays
  float array_ht_body_center[4][4];
  Map<Matrix4fRM>(&array_ht_body_center[0][0], 4, 4) = ht_body_center;
  float array_ht_bcXlrb[4][4];
  array_htLegRightBack(array_ht_body_center, body_length, body_width, array_ht_bcXlrb);
  return Map<Matrix4fRM>(&array_ht_bcXlrb[0][0], 4, 4);
#endif //ARRAY_IMPLEMENTATION
}

Matrix4f htLegRightFront(const Matrix4f& ht_body_center, float body_length, float body_width) {
#ifndef ARRAY_IMPLEMENTATION
  // Build up matrix representing right front leg ht. First, a pi/2 rotation in y
  Matrix4f htLegRightBack = homogRotXyz(0.0f, M_PI/2.0f, 0.0);

  // Next, add the linear translation portion
  htLegRightBack.block<3,1>(0,3) = Vector3f(body_length/2.0f, 0.0f, body_width/2.0f);

  // Return the matrix product of ht_body_center and the leg ht
  return (ht_body_center * htLegRightBack);
#else // ARRAY_IMPLEMENTATION
  // implement using arrays
  float array_ht_body_center[4][4];
  Map<Matrix4fRM>(&array_ht_body_center[0][0], 4, 4) = ht_body_center;
  float array_ht_bcXleg[4][4];
  array_htLegRightFront(array_ht_body_center, body_length, body_width, array_ht_bcXleg);
  return Map<Matrix4fRM>(&array_ht_bcXleg[0][0], 4, 4);
#endif //ARRAY_IMPLEMENTATION
}

Matrix4f htLegLeftFront(const Matrix4f& ht_body_center, float body_length, float body_width) {
#ifndef ARRAY_IMPLEMENTATION
  // Build up matrix representing right front leg ht. First, a pi/2 rotation in y
  Matrix4f htLegLeftFront = homogRotXyz(0.0f, -M_PI/2.0f, 0.0);

  // Next, add the linear translation portion
  htLegLeftFront.block<3,1>(0,3) = Vector3f(body_length/2.0f, 0.0f, -body_width/2.0f);

  // Return the matrix product of ht_body_center and the leg ht
  return (ht_body_center * htLegLeftFront);
#else // ARRAY_IMPLEMENTATION
  // implement using arrays
  float array_ht_body_center[4][4];
  Map<Matrix4fRM>(&array_ht_body_center[0][0], 4, 4) = ht_body_center;
  float array_ht_bcXleg[4][4];
  array_htLegLeftFront(array_ht_body_center, body_length, body_width, array_ht_bcXleg);
  return Map<Matrix4fRM>(&array_ht_bcXleg[0][0], 4, 4);
#endif //ARRAY_IMPLEMENTATION
}

Matrix4f htLegLeftBack(const Matrix4f& ht_body_center, float body_length, float body_width) {
#ifndef ARRAY_IMPLEMENTATION
  // Build up matrix representing right back leg ht. First, a pi/2 rotation in y
  Matrix4f htLegLeftBack = homogRotXyz(0.0f, -M_PI/2.0f, 0.0);

  // Next, add the linear translation portion
  htLegLeftBack.block<3,1>(0,3) = Vector3f(-body_length/2.0f, 0.0f, -body_width/2.0f);

  // Return the matrix product of ht_body_center and the leg ht
  return (ht_body_center * htLegLeftBack);
#else // ARRAY_IMPLEMENTATION
  // implement using arrays
  float array_ht_body_center[4][4];
  Map<Matrix4fRM>(&array_ht_body_center[0][0], 4, 4) = ht_body_center;
  float array_ht_bcXleg[4][4];
  array_htLegLeftBack(array_ht_body_center, body_length, body_width, array_ht_bcXleg);
  return Map<Matrix4fRM>(&array_ht_bcXleg[0][0], 4, 4);
#endif //ARRAY_IMPLEMENTATION
}

#ifdef ARRAY_IMPLEMENTATION
static void array_ht0To1(float rot_ang, float link_length, float ht_0_to_1[][4]) {
  
  // Build up the matrix as from the paper
  array_homogRotXyz(0.0f, 0.0f, rot_ang, ht_0_to_1);

  // Add in remaining terms
  ht_0_to_1[0][3] = -link_length*cos(rot_ang);
  ht_0_to_1[1][3] = -link_length*sin(rot_ang);

  // return ht_0_to_1;
}

static void array_ht2To3(float rot_ang, float link_length, float ht_2_to_3[][4]) {
  
  // Build up the matrix as from the paper
  array_homogRotXyz(0.0f, 0.0f, rot_ang, ht_2_to_3);

  // Add in remaining terms
  ht_2_to_3[0][3] = link_length*cos(rot_ang);
  ht_2_to_3[1][3] = link_length*sin(rot_ang);

  // return ht_0_to_1;
}

#define array_ht3To4 array_ht2To3
void array_ht0To4(const JointAngles& joint_angles,
                const LinkLengths& link_lengths,
                float ht_0_to_4[][4]) {
  float ht_0_to_1[4][4];
  array_ht0To1(joint_angles.ang1, link_lengths.l1, ht_0_to_1);
  float ht_1_to_2[4][4] = {
      {0.0f,   0.0f,   -1.0f,   0.0f},
     {-1.0f,   0.0f,    0.0f,   0.0f},
      {0.0f,   1.0f,    0.0f,   0.0f},
      {0.0f,   0.0f,    0.0f,   1.0f}};
  float ht_2_to_3[4][4];
  array_ht2To3(joint_angles.ang2, link_lengths.l2, ht_2_to_3);
  float ht_3_to_4[4][4];
  array_ht3To4(joint_angles.ang3,  link_lengths.l3, ht_3_to_4);

  float ht_1_to_2_out[4][4];
  dspm_mult_f32_ae32((float*) ht_0_to_1, (float*) ht_1_to_2, (float*) ht_1_to_2_out, 4, 4, 4);
  float ht_2_to_3_out[4][4];
  dspm_mult_f32_ae32((float*) ht_1_to_2_out, (float*) ht_2_to_3, (float*) ht_2_to_3_out, 4, 4, 4);
  dspm_mult_f32_ae32((float*) ht_2_to_3_out, (float*) ht_3_to_4, (float*) ht_0_to_4, 4, 4, 4);
}
#endif //ARRAY_IMPLEMENTATION

Matrix4f ht0To1(float rot_ang, float link_length) {
#ifndef ARRAY_IMPLEMENTATION
  // Build up the matrix as from the paper
  Matrix4f ht_0_to_1 = homogRotXyz(0.0f, 0.0f, rot_ang);

  // Add in remaining terms
  ht_0_to_1(0,3) = -link_length*cos(rot_ang);
  ht_0_to_1(1,3) = -link_length*sin(rot_ang);

  return ht_0_to_1;
#else // ARRAY_IMPLEMENTATION
  float ht_0_to_1[4][4] = {0,};
  array_ht0To1(rot_ang, link_length, ht_0_to_1);
  return Map<Matrix4fRM>(&ht_0_to_1[0][0], 4, 4);
#endif //ARRAY_IMPLEMENTATION
}

Matrix4f ht1To2() {
  // Build up the matrix as from the paper
  Matrix4f ht_1_to_2;

  ht_1_to_2 <<
      0.0f,   0.0f,   -1.0f,   0.0f,
     -1.0f,   0.0f,    0.0f,   0.0f,
      0.0f,   1.0f,    0.0f,   0.0f,
      0.0f,   0.0f,    0.0f,   1.0f;

  return ht_1_to_2;
}

Matrix4f ht2To3(float rot_ang, float link_length) {
#ifndef ARRAY_IMPLEMENTATION
  // Build up the matrix as from the paper
  Matrix4f ht_2_to_3 = homogRotXyz(0.0f, 0.0f, rot_ang);

  // Add in remaining terms
  ht_2_to_3(0,3) = link_length*cos(rot_ang);
  ht_2_to_3(1,3) = link_length*sin(rot_ang);

  return ht_2_to_3;
#else // ARRAY_IMPLEMENTATION
  float ht_2_to_3[4][4] = {0,};
  array_ht2To3(rot_ang, link_length, ht_2_to_3);
  return Map<Matrix4fRM>(&ht_2_to_3[0][0], 4, 4);
#endif //ARRAY_IMPLEMENTATION
}

Matrix4f ht3To4(float rot_ang, float link_length) {
  // Same as the 2 to 3 transformation, so just call that function
  
  return ht2To3(rot_ang, link_length);
}

Matrix4f ht0To4(const JointAngles& joint_angles,
                const LinkLengths& link_lengths) {
  // Result is a sequential multiplication of all 4 transform matrices
#ifndef ARRAY_IMPLEMENTATION
  return (ht0To1(joint_angles.ang1, link_lengths.l1) *
          ht1To2() *
          ht2To3(joint_angles.ang2, link_lengths.l2) *
          ht3To4(joint_angles.ang3,  link_lengths.l3));
#else // ARRAY_IMPLEMENTATION
  float ht_0_to_4[4][4];
  array_ht0To4(joint_angles, link_lengths, ht_0_to_4);

  return (Map<Matrix4fRM>(&ht_0_to_4[0][0], 4, 4));
#endif //ARRAY_IMPLEMENTATION
}


JointAngles ikine(const Point& point, const LinkLengths& link_lengths, bool is_leg_12) {
  using namespace std;

  // Initialize return struct
  JointAngles joint_angles;

  // Convenience variables for math
  float x4 = point.x;
  float y4 = point.y;
  float z4 = point.z;
  float l1 = link_lengths.l1;
  float l2 = link_lengths.l2;
  float l3 = link_lengths.l3;
  
  // Supporting variable D
  float D = (x4*x4 + y4*y4 + z4*z4 - l1*l1 - l2*l2 - l3*l3) /
            (2*l2*l3);
  
  // Poor man's inverse kinematics reachability protection:
  // Limit D to a maximum value of 1, otherwise the square root functions
  // below (sqrt(1 - D^2)) will attempt a square root of a negative number
  if (D > 1.0f) {
    D = 1.0f;
  } else if (D < -1.0f) {
    D = -1.0f;
  }

  if (is_leg_12) {
    joint_angles.ang3 = atan2(sqrt(1 - D*D), D);
  } else {
    joint_angles.ang3 = atan2(-sqrt(1 - D*D), D);
  }

  // Another poor mans reachability sqrt protection
  float protected_sqrt_val = x4*x4 + y4*y4 - l1*l1;
  if (protected_sqrt_val < 0.0f) { protected_sqrt_val = 0.0f;}

  joint_angles.ang2 = atan2(z4, sqrt(protected_sqrt_val)) -
         atan2(l3*sin(joint_angles.ang3), l2 + l3*cos(joint_angles.ang3));

  joint_angles.ang1 = atan2(y4, x4) + atan2(sqrt(protected_sqrt_val), -l1);

  return joint_angles;
} 

#ifdef ARRAY_IMPLEMENTATION
#ifndef ESP32_DSP
// dumb implementation of array multiply for x86
int dspm_mult_f32_ansi(const float *A, const float *B, float *C, int m, int n, int k)
{
    for (int i = 0 ; i < m ; i++) {
        for (int j = 0 ; j < k ; j++) {
            C[i * k + j] = A[i * n] * B[j];
            for (int s = 1; s < n ; s++) {
                C[i * k + j] += A[i * n + s] * B[s * k + j];
            }
        }
    }
    return 0;
}
void dsps_add_f32_ansi(const float *input1, const float *input2, float *output, int len, int step1, int step2, int step_out)
{
  for (int i = 0; i < len; ++i) {
    output[i*step_out] = input1[i*step1] + input2[i*step2];
  }
}
#endif // ESP32_DSP

void array_homogRotXyz(float x_ang, float y_ang, float z_ang, float Rxyz[][4])
{
  // Create 3d transformation, and execute x, y, and z rotations
  float cos_x_ang = cos(x_ang); float sin_x_ang = sin(x_ang);
  float Rx[4][4] = {
                    {1.0f, 0.0f, 0.0f, 0.0f}, 
                    {0.0f, cos_x_ang, -sin_x_ang, 0.0f}, 
                    {0.0f, sin_x_ang, cos_x_ang, 0.0f}, 
                    {0.0f, 0.0f, 0.0f, 1.0f}}; // X-axis rotation matrix
  float cos_y_ang   = cos(y_ang); float sin_y_ang   = sin(y_ang);
  float Ry[4][4] = {
                    {cos_y_ang, 0.0f, sin_y_ang, 0.0f},
                    {0.0f, 1.0f, 0.0f, 0.0f},
                    {-sin_y_ang, 0.0f, cos_y_ang, 0.0f},
                    {0.0f, 0.0f, 0.0f, 1.0f}}; // Y-axis rotation matrix
  float cos_z_ang = cos(z_ang); float sin_z_ang = sin(z_ang);
  float Rz[4][4] = {
                    {cos_z_ang, -sin_z_ang, 0, 0},
                    {sin_z_ang, cos_z_ang, 0, 0},
                    {0.0f, 0.0f, 1.0f, 0.0f},
                    {0.0f, 0.0f, 0.0f, 1.0f}}; // Z-axis rotation matrix
  
  float Rxy[4][4];
  dspm_mult_f32_ae32((float*) Rx, (float*) Ry, (float*) Rxy, 4, 4, 4);
  dspm_mult_f32_ae32((float*) Rxy, (float*) Rz, (float*) Rxyz, 4, 4, 4);
}

void array_homogTransXyz(float x, float y, float z, float Txyz[][4])
{
  // Create a linear translation homogenous transformation matrix
  Txyz[0][3] = x; Txyz[1][3] = y; Txyz[2][3] = z;
}

void array_homogInverse(const float a[][4], float b[][4])
{
   float s0 = a[0][0] * a[1][1] - a[1][0] * a[0][1];
    float s1 = a[0][0] * a[1][2] - a[1][0] * a[0][2];
    float s2 = a[0][0] * a[1][3] - a[1][0] * a[0][3];
    float s3 = a[0][1] * a[1][2] - a[1][1] * a[0][2];
    float s4 = a[0][1] * a[1][3] - a[1][1] * a[0][3];
    float s5 = a[0][2] * a[1][3] - a[1][2] * a[0][3];

    float c5 = a[2][2] * a[3][3] - a[3][2] * a[2][3];
    float c4 = a[2][1] * a[3][3] - a[3][1] * a[2][3];
    float c3 = a[2][1] * a[3][2] - a[3][1] * a[2][2];
    float c2 = a[2][0] * a[3][3] - a[3][0] * a[2][3];
    float c1 = a[2][0] * a[3][2] - a[3][0] * a[2][2];
    float c0 = a[2][0] * a[3][1] - a[3][0] * a[2][1];

    // Should check for 0 determinant
    float det = (s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0);

    if (det == 0.0) return;

    float invdet = 1.0 / det;


    b[0][0] = ( a[1][1] * c5 - a[1][2] * c4 + a[1][3] * c3) * invdet;
    b[0][1] = (-a[0][1] * c5 + a[0][2] * c4 - a[0][3] * c3) * invdet;
    b[0][2] = ( a[3][1] * s5 - a[3][2] * s4 + a[3][3] * s3) * invdet;
    b[0][3] = (-a[2][1] * s5 + a[2][2] * s4 - a[2][3] * s3) * invdet;

    b[1][0] = (-a[1][0] * c5 + a[1][2] * c2 - a[1][3] * c1) * invdet;
    b[1][1] = ( a[0][0] * c5 - a[0][2] * c2 + a[0][3] * c1) * invdet;
    b[1][2] = (-a[3][0] * s5 + a[3][2] * s2 - a[3][3] * s1) * invdet;
    b[1][3] = ( a[2][0] * s5 - a[2][2] * s2 + a[2][3] * s1) * invdet;

    b[2][0] = ( a[1][0] * c4 - a[1][1] * c2 + a[1][3] * c0) * invdet;
    b[2][1] = (-a[0][0] * c4 + a[0][1] * c2 - a[0][3] * c0) * invdet;
    b[2][2] = ( a[3][0] * s4 - a[3][1] * s2 + a[3][3] * s0) * invdet;
    b[2][3] = (-a[2][0] * s4 + a[2][1] * s2 - a[2][3] * s0) * invdet;

    b[3][0] = (-a[1][0] * c3 + a[1][1] * c1 - a[1][2] * c0) * invdet;
    b[3][1] = ( a[0][0] * c3 - a[0][1] * c1 + a[0][2] * c0) * invdet;
    b[3][2] = (-a[3][0] * s3 + a[3][1] * s1 - a[3][2] * s0) * invdet;
    b[3][3] = ( a[2][0] * s3 - a[2][1] * s1 + a[2][2] * s0) * invdet;
}

void array_htLegRightBack(const float ht_body_center[][4], float body_length, float body_width, float ret[][4]) {

  // Build up matrix representing right back leg ht. First, a pi/2 rotation in y
  float htLeg[4][4] = {0,};
  // std::cout << "homogRotXyz: \n" << homogRotXyz(0.0f, M_PI/2.0f, 0.0) << "\n";
  array_homogRotXyz(0.0f, M_PI/2.0f, 0.0, htLeg);

  // Next, add the linear translation portion
  htLeg[0][3] = -body_length/2.0f;
  htLeg[2][3] = body_width/2.0f;

  // Return the matrix product of ht_body_center and the leg rightback ht
  dspm_mult_f32_ae32((float*) ht_body_center, (float*) htLeg, (float*) ret, 4, 4, 4);
}

void array_htLegRightFront(const float ht_body_center[][4], float body_length, float body_width, float ret[][4]) {

  // Build up matrix representing right back leg ht. First, a pi/2 rotation in y
  float htLeg[4][4] = {0,};
  // std::cout << "homogRotXyz: \n" << homogRotXyz(0.0f, M_PI/2.0f, 0.0) << "\n";
  array_homogRotXyz(0.0f, M_PI/2.0f, 0.0, htLeg);

  // Next, add the linear translation portion
  htLeg[0][3] = body_length/2.0f;
  htLeg[2][3] = body_width/2.0f;

  // Return the matrix product of ht_body_center and the leg rightback ht
  dspm_mult_f32_ae32((float*) ht_body_center, (float*) htLeg, (float*) ret, 4, 4, 4);
}

void array_htLegLeftFront(const float ht_body_center[][4], float body_length, float body_width, float ret[][4]) {

  // Build up matrix representing right back leg ht. First, a pi/2 rotation in y
  float htLeg[4][4] = {0,};
  // std::cout << "homogRotXyz: \n" << homogRotXyz(0.0f, M_PI/2.0f, 0.0) << "\n";
  array_homogRotXyz(0.0f, -M_PI/2.0f, 0.0, htLeg);

  // Next, add the linear translation portion
  htLeg[0][3] = body_length/2.0f;
  htLeg[2][3] = -body_width/2.0f;

  // Return the matrix product of ht_body_center and the leg rightback ht
  dspm_mult_f32_ae32((float*) ht_body_center, (float*) htLeg, (float*) ret, 4, 4, 4);
}

void array_htLegLeftBack(const float ht_body_center[][4], float body_length, float body_width, float ret[][4]) {

  // Build up matrix representing right back leg ht. First, a pi/2 rotation in y
  float htLeg[4][4] = {0,};
  // std::cout << "homogRotXyz: \n" << homogRotXyz(0.0f, M_PI/2.0f, 0.0) << "\n";
  array_homogRotXyz(0.0f, -M_PI/2.0f, 0.0, htLeg);

  // Next, add the linear translation portion
  htLeg[0][3] = -body_length/2.0f;
  htLeg[2][3] = -body_width/2.0f;

  // Return the matrix product of ht_body_center and the leg rightback ht
  dspm_mult_f32_ae32((float*) ht_body_center, (float*) htLeg, (float*) ret, 4, 4, 4);
}
#endif //ARRAY_IMPLEMENTATION
}
