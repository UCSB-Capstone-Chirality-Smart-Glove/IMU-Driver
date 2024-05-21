#include "hand.h"

void calibrate_hand_basis(Hand* hand) {
    hand->basis[0] = (vec3) {1, 0, 0};
    hand->basis[1] = (vec3) {0, 1, 0};
    hand->basis[2] = (vec3) {0, 0, 1};
}

void update_hand(Hand* hand, IMUData* hand_rotation, int16_t frequency, FingerSensorData finger_data[5], float flex_data[]) {
	PDEBUG("hand Roll: %d\n", (int)hand_rotation->gyro.roll);
	PDEBUG("hand Pitch: %d\n", (int)hand_rotation->gyro.pitch);
	PDEBUG("hand Yaw: %d\n", (int)hand_rotation->gyro.yaw);
    for (int i = 0; i < 1; i++) {
        // apply finger rotation to fingers
//    	PDEBUG("in update_hand | x:%f\n", finger_data[i].base.accel.x);
//    	PDEBUG("in update hand | y:%f\n", finger_data[0].base.accel.y);
//    	PDEBUG("in update hand | z:%f\n", finger_data[0].base.accel.z);
        update_finger(hand->finger[i], &(finger_data[i]), frequency, hand_rotation);
    }
    // TODO: update thumb
    update_thumb(hand->thumb, &(finger_data[4]), frequency, flex_data);

    // update hand basis
    vec3 gyro_rotation_matrix[3];
    fill_rotation_matrix(hand_rotation->gyro, frequency, gyro_rotation_matrix);

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

void initialize_hand(Hand* hand, IMUData* hand_sensor_data, FingerSensorData finger_sensor_data[4]){
	// initialize hand basis
	hand->basis[0] = (vec3) {1, 0, 0};
	hand->basis[1] = (vec3) {0, 1, 0};
	hand->basis[2] = (vec3) {0, 0, 1};

	//initialize hand rotation data
	*hand_sensor_data = (IMUData){
		.gyro = (rotation_vec3) {
			.roll = 0,
			.pitch = 0,
			.yaw = 0
		},
		.accel = (vec3) {
			.x = 0,
			.y = 0,
			.z = 0
		}

	};

	// initialize fingers
	for (int i = 0; i < 4; i++) {
		hand->finger[i] = (Finger*) malloc(sizeof(Finger));
		initialize_finger(hand->finger[i], &(finger_sensor_data[i]));
	}
}
