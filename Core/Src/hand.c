#include "hand.h"

void calibrate_hand_basis(Hand* hand) {
    hand->basis[0] = (vec3) {1, 0, 0};
    hand->basis[1] = (vec3) {0, 1, 0};
    hand->basis[2] = (vec3) {0, 0, 1};
}

void update_hand(Hand* hand, rotation_vec3 hand_rotation, int16_t frequency, FingerSensorData finger_data[4]) {
    for (int i = 0; i < 4; i++) {
        // apply finger rotation to fingers
        update_finger(&(hand->finger[i]), &finger_data[i], frequency, hand_rotation);
    }

    // TODO: update web angles

    // TODO: update thumb

    // update hand basis
    vec3 gyro_rotation_matrix[3];
    fill_rotation_matrix(hand_rotation, frequency, gyro_rotation_matrix);

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
    for (int i=0; i<=2; i++) {
        temp[i] = multiply_vector_over_matrix(rotation_matrix, hand->basis[i]);
    }
    for (int i = 0; i < 3; i++) hand->basis[i] = temp[i];
}

void InitializeHand(Finger *finger,
		vec3 *hand_basis,
		rotation_vec3 *base_data,
		rotation_vec3 *tip_data,
		FingerSensorData *finger_sensor_data
		) {
	*finger = (Finger) {
		.curl = 0,
		.bend = 0
	};

	hand_basis[0] = (vec3) {1, 0, 0};
	hand_basis[1] = (vec3) {0, 1, 0};
	hand_basis[2] = (vec3) {0, 0, 1};

	*base_data = (rotation_vec3) {
		.roll = 0,
		.pitch = 0,
		.yaw = 0
	};
	*tip_data = (rotation_vec3) {
		.roll = 0,
		.pitch = 0,
		.yaw = 0
	};


	*finger_sensor_data = (FingerSensorData) {
			.base = *base_data,
			.tip = *tip_data
	};
}
