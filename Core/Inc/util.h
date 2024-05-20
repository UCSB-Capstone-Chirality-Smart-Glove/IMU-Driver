#ifndef UTIL_HAND
#define UTIL_HAND

#include <stdlib.h>
#include <stdint.h>
#include <math.h>

// rotation vector based on gyroscope x, y and z from Bosch BMI 323
//
// roll is a measure of counter-clockwise rotation around the x axis
// pitch is the same but around the y axis, and yaw is around the z axis
typedef struct {
    float roll, pitch, yaw;
} rotation_vec3;

// cartesian vector 
typedef struct {
    float x, y, z;
} vec3;

//structure to encapsulate full sensor data
typedef struct {
	rotation_vec3 gyro;
	vec3 accel;
} IMUData;
// multiplies a 3x1 vector over a 3x3 matrix
// - matrix: a 3-long array of vec3's
// - vector: a vec3
// Note: each vec3 in a matrix represents a column
vec3 multiply_vector_over_matrix(vec3 matrix[3], vec3 vector);

// takes an angular rate vector and a frequency, and fills the passed vec3 array with the corresponding rotation matrix
// allocate the passed vec3 array on the stack and pass the address to matrix
// note: rotations are expected in degrees
// Reference: https://en.wikipedia.org/wiki/Rotation_matrix
void fill_rotation_matrix(rotation_vec3 rotation, float frequency, vec3 matrix[3]);

// takes an old and new basis, and fills the passed vec3 array with the corresponding change basis matrix
// allocate the passed vec3 array on the stack and pass the address to matrix
void fill_change_basis_matrix(vec3 old[3], vec3 new[3], vec3 matrix[3]);

// transposes a 3-long vec3 array in place
void transpose_matrix(vec3 matrix[3]);

// takes element-wise weighted average of passed matrix array and stores it in result
void average_matrices(vec3 **matrices, float *weights, int length, vec3 result[3]);

rotation_vec3 matrix_to_euler(vec3 matrix[3]);

float magnitude(vec3 v);

void L2_vec_norm(vec3* v);

float dot_vec3(vec3 v1, vec3 v2);

// maximum angular rate of BMI323; set manually
#define angular_rate_max 500.0

// conversion rate of LSB form to degree form
extern float LSB_to_degrees;

// the smallest interval present in the trig lookup tables
extern int ANGLE_RESOLUTION;

// precomputed lookup table for sines (in degrees)
extern float sine[360];
extern float cosine[360];
#endif
