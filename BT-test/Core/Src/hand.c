#include "hand.h"

void calibrate_hand_basis(Hand* hand) {
    hand->basis[0] = (vec3) {1, 0, 0};
    hand->basis[1] = (vec3) {0, 1, 0};
    hand->basis[2] = (vec3) {0, 0, 1};
}

void update_hand(Hand* hand, rotation_vec3 hand_rotation, int16_t frequency, FingerSensorData finger_data[4]) {
    vec3 rotation_matrix[3];
    fill_rotation_matrix(hand_rotation, frequency, rotation_matrix);

    for (int i = 0; i < 4; i++) {
        // apply finger rotation to fingers
        update_finger(&(hand->finger[i]), &finger_data[i], frequency, hand->basis);
    }

    // TODO: update thumb

    // update hand basis
    vec3 temp[3];
    for (int i=0; i<=2; i++) {
        temp[i] = multiply_vector_over_matrix(rotation_matrix, hand->basis[i]);
    }
    for (int i = 0; i < 3; i++) hand->basis[i] = temp[i];
}
