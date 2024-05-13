#include <math.h>
#include "finger.h"


float get_bend(IMUData* hand_data, IMUData* base_data, int16_t frequency){
	float gyro_bend = (hand_data->gyro.roll - base_data->gyro.roll)/frequency;
	float difference_ratio = (hand_data->gyro.roll - base_data->gyro.roll)/((hand_data->gyro.roll + base_data->gyro.roll)/2);
//	PDEBUG("hand data roll: %d\n", (int)hand_data.roll);
//	PDEBUG("base data roll: %d\n", (int)base_data.roll);
//	PDEBUG("gyro bend change: %d\n", (int)gyro_bend);
    if (difference_ratio < 0.025) return gyro_bend;
    return 0;
}

float get_curl(FingerSensorData* finger_data, int16_t frequency) {
	float gyro_curl = (finger_data->base.gyro.roll - finger_data->tip.gyro.roll)/frequency;
//	PDEBUG("base roll: %d\n", (int)finger_data->base.roll);
//	PDEBUG("tip roll: %d\n", (int)finger_data->tip.roll);
	PDEBUG("gyro curl change: %d\n", (int)gyro_curl);
    return gyro_curl;
}

void calibrate_finger(Finger* finger) {
	finger->bend = 0;
	finger->curl = 0;
}

void generate_gyroscope_update_matrix(Finger* finger, FingerSensorData* finger_data, int16_t frequency, vec3 hand_basis[3], vec3 result[3]) {
    fill_rotation_matrix(finger_data->base.gyro, frequency, result);
}

void update_finger(Finger* finger, FingerSensorData* finger_data, int16_t frequency, IMUData* hand_data) {
    // frequency adjustment factor
    const float adjustment = 1.5;

	// update bend
    float bend_change = get_bend(hand_data, &(finger_data->base), frequency);
    if (bend_change < 150) finger->bend += bend_change;
    finger->bend = fmod(finger->bend, 360);
	PDEBUG("gyro bend: %d\n", (int)finger->bend);

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

void update_thumb(Thumb* thumb, FingerSensorData* finger_data, int16_t knuckle_rotation_change, int16_t frequency, IMUData* hand_data) {
    // could probably do some sensor fusion here to make the finger data more accurate
//    update_finger(&(thumb->finger), finger_data, frequency, hand_data);
	// update bend (using palm flex sensor)

	// update curl (with IMUs)

	// update web angle (with flex sensor)
    thumb->web_angle += knuckle_rotation_change;
}

void initialize_finger(Finger *finger,
		FingerSensorData *finger_sensor_data
		) {
	*finger = (Finger) {
		.curl = 0,
		.bend = 0
	};

	IMUData base_data = {
			(rotation_vec3) {
				.roll = 0,
				.pitch = 0,
				.yaw = 0
			},
			(vec3) {
				.x = 0,
				.y = 0,
				.z = 0
			}
	};

	IMUData tip_data = {
			(rotation_vec3) {
				.roll = 0,
				.pitch = 0,
				.yaw = 0
			},
			(vec3) {
				.x = 0,
				.y = 0,
				.z = 0
			}
	};


	*finger_sensor_data = (FingerSensorData) {
			.base = base_data,
			.tip = tip_data
	};
}
