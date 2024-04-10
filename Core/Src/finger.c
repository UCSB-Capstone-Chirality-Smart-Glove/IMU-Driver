#include <math.h>
#include "finger.h"

int16_t get_bend(FingerSensorData* finger_data, int16_t frequency) {
    return (finger_data->base.roll - finger_data->tip.roll)/frequency;
}

void calibrate_finger(Finger* finger) {
    finger->basis[0] = (vec3) {1, 0, 0};
    finger->basis[1] = (vec3) {0, 1, 0};
    finger->basis[2] = (vec3) {0, 0, 1};
}

void generate_gyroscope_update_matrix(Finger* finger, FingerSensorData* finger_data, int16_t frequency, vec3 hand_basis[3], vec3 result[3]) {
    fill_rotation_matrix(finger_data->base, frequency, result);
}

void update_finger(Finger* finger, FingerSensorData* finger_data, int16_t frequency, vec3 hand_basis[3]) {
    // create gyroscope update matrix
    vec3 gyro_rotation_matrix[3];
    fill_rotation_matrix(finger_data->base, frequency, gyro_rotation_matrix);

    // TODO: create accelerometer update matrix
    // stub:
    vec3 accel_rotation_matrix[3];
    accel_rotation_matrix[0] = (vec3) {1, 0, 0};
    accel_rotation_matrix[1] = (vec3) {0, 1, 0};
    accel_rotation_matrix[2] = (vec3) {0, 0, 1};

    // take weighted average of matrices
    float gyro_weight = 1;
    float accel_weight = 0;
    float weights[2] = {gyro_weight, accel_weight};
    vec3 *matrices[2] = {gyro_rotation_matrix, accel_rotation_matrix};
    vec3 rotation_matrix[3];
    average_matrices(matrices, weights, 2, rotation_matrix);

    vec3 temp[3];
    // apply rotation
	for (int i = 0; i < 3; i++) {
		temp[i] = multiply_vector_over_matrix(finger->basis, rotation_matrix[i]);
	}
	for (int i = 0; i < 3; i++) finger->basis[i] = temp[i];

    // update bend
    int16_t bend_change = get_bend(finger_data, frequency);
    finger->bend += bend_change;
}

void calibrate_thumb(Thumb* thumb) {
    calibrate_finger(&(thumb->finger));
    thumb->knuckle_rotation = 0;
}

void update_thumb(Thumb* thumb, FingerSensorData* finger_data, int16_t knuckle_rotation_change, int16_t frequency, vec3* hand_basis) {
    // could probably do some sensor fusion here to make the finger data more accurate
    update_finger(&(thumb->finger), finger_data, frequency, hand_basis);
    thumb->knuckle_rotation += knuckle_rotation_change;
}
