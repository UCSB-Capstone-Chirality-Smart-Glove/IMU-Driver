#include <math.h>
#include "finger.h"


float get_bend(rotation_vec3 hand_data, rotation_vec3 base_data, int16_t frequency) {
    return (hand_data.roll - base_data.roll)/frequency;
}

float get_curl(FingerSensorData* finger_data, int16_t frequency) {
	PDEBUG("base roll: %d\n", (int)finger_data->base.roll);
	PDEBUG("tip roll: %d\n", (int)finger_data->tip.roll);
    return (finger_data->base.roll - finger_data->tip.roll)/frequency;
}

void calibrate_finger(Finger* finger) {
//    finger->basis[0] = (vec3) {1, 0, 0};
//    finger->basis[1] = (vec3) {0, 1, 0};
//    finger->basis[2] = (vec3) {0, 0, 1};
	finger->bend = 0;
	finger->curl = 0;
}

void generate_gyroscope_update_matrix(Finger* finger, FingerSensorData* finger_data, int16_t frequency, vec3 hand_basis[3], vec3 result[3]) {
    fill_rotation_matrix(finger_data->base, frequency, result);
}

void update_finger(Finger* finger, FingerSensorData* finger_data, int16_t frequency, rotation_vec3* hand_data) {
    // frequency adjustment factor
    const float adjustment = 1.5;

	// update bend
    float bend_change = get_bend(*hand_data, finger_data->base, frequency);
    finger->bend += bend_change * adjustment;
    finger->bend = fmod(finger->bend, 360);

    // update curl
    float curl_change = get_curl(finger_data, frequency);
	PDEBUG("Curl change: %d\n", (int)curl_change);
    if (fabs(curl_change) < 150) finger->curl += curl_change;
    finger->curl = fmod(finger->curl, 360);
}

void calibrate_thumb(Thumb* thumb) {
    calibrate_finger(&(thumb->finger));
    thumb->web_angle = 0;
}

void update_thumb(Thumb* thumb, FingerSensorData* finger_data, int16_t knuckle_rotation_change, int16_t frequency, rotation_vec3* hand_data) {
    // could probably do some sensor fusion here to make the finger data more accurate
    update_finger(&(thumb->finger), finger_data, frequency, hand_data);
    thumb->web_angle += knuckle_rotation_change;
}

void initialize_finger(Finger *finger,
		FingerSensorData *finger_sensor_data
		) {
	*finger = (Finger) {
		.curl = 0,
		.bend = 0
	};

	rotation_vec3 base_data = (rotation_vec3) {
		.roll = 0,
		.pitch = 0,
		.yaw = 0
	};
	rotation_vec3 tip_data = (rotation_vec3) {
		.roll = 0,
		.pitch = 0,
		.yaw = 0
	};


	*finger_sensor_data = (FingerSensorData) {
			.base = base_data,
			.tip = tip_data
	};
}
