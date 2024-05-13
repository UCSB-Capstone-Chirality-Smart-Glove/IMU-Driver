#include <math.h>
#include "finger.h"

float accel_relative_rotation_from_gravity(vec3 lower_gravity_vector, vec3 upper_gravity_vector){
    L2_vec_norm(&lower_gravity_vector);
    L2_vec_norm(&upper_gravity_vector);
    // # Step 3: Calculate Rotation Angle
    return (180/M_PI)*((float)acos(dot_vec3(lower_gravity_vector, upper_gravity_vector)));
}

/*
* TODO: have to figure out magnitude of IMU accel in gs (gravity)
*/
float get_fusion_bendcurl(vec3 gravity_vector, float gyro_bendcurl, float accel_bendcurl){
    /* Two factors to account for:
    * 1) Magnitude: if magnitude of acceleration is high, accelerometer is less dependable
    * 2) Orientation: if accelerometer is expected to move orthogonal to gravity (gravity vector alined with x-axis), then acceleromter is less dependable
    *
    * In general, gyroscope is more dependable with respect to these factors in deriving rotational position. When both sensors are equally dependable, we want
    * reading from both sensors to be equally weighted (0.5 each). Weights should always add up to 1.
    */

    // perform magnitude-based weighting
    float mag = magnitude(gravity_vector);
    float R = (10*PEAK_CONST)/((mag-1)*(mag-1) + PEAK_CONST);
    float mag_gyro_weight = Q/(R+Q);
    float mag_accel_weight = R/(R+Q);

    // perform orientation-based weighting
    L2_vec_norm(&gravity_vector);
    float cos = gravity_vector.x; //weight is the cos of the angle between (1 if gravity aligned with x axis, 0 if orthogonal to x-axis)
    float orthog_accel_weight = sqrt(1 - cos*cos); // get the sine of the angle cos^2 = 1 - sin^2 and (ignore rest for now)then scale by 0.5 to get a maximum of 0.5 (since gyroscope is more dependable here)

    // float orthog_gyro_weight = 1 - 0.5*orthog_accel_weight;
    // printf("ortho_gyro_weight: %f, ortho_accel_weight: %f\n", orthog_gyro_weight, 0.5*orthog_accel_weight);
    // printf("mag_gyro_weight: %f, mag_accel_weight: %f\n", mag_gyro_weight, mag_accel_weight);

    // float ortho_filtered = 0.5*orthog_accel_weight*accel_bendcurl + orthog_gyro_weight*gyro_bendcurl;
    // float mag_filtered = mag_accel_weight*accel_bendcurl + mag_gyro_weight*gyro_bendcurl;
    // printf("ortho_filtered: %f\n", ortho_filtered);
    // printf("mag_filtered: %f\n", mag_filtered);
    // printf("naive_avg_angle: %f\n", (mag_filtered+ortho_filtered)/2);

    float accel_weight = orthog_accel_weight*mag_accel_weight;
    float gyro_weight = 1 - accel_weight;
    float weighted_angle = gyro_weight*gyro_bendcurl + accel_weight*accel_bendcurl;
    // printf("gyro_weight: %f, accel_weight: %f\n", gyro_weight, orthog_accel_weight*mag_accel_weight);

    return weighted_angle;//accel_weight*accel_bendcurl + gyro_weight*gyro_bendcurl;
}

float get_bend(IMUData* hand_data, IMUData* base_data, int16_t frequency){
	float gyro_bend = (hand_data->gyro.roll - base_data->gyro.roll)/frequency;
	float accel_bend = accel_relative_rotation_from_gravity(hand_data->accel, base_data->accel);
	float difference_ratio = (hand_data->gyro.roll - base_data->gyro.roll)/((hand_data->gyro.roll + base_data->gyro.roll)/2);
	float weighted_bend = get_fusion_bendcurl(base_data->accel, gyro_bend, accel_bend);
//	PDEBUG("hand data roll: %d\n", (int)hand_data.roll);
//	PDEBUG("base data roll: %d\n", (int)base_data.roll);
//	PDEBUG("gyro bend change: %d\n", (int)gyro_bend);
    if (difference_ratio < 0.025) return weighted_bend;
    return 0;
}

float get_curl(FingerSensorData* finger_data, int16_t frequency) {
	float gyro_curl = (finger_data->base.gyro.roll - finger_data->tip.gyro.roll)/frequency;
	float accel_curl = accel_relative_rotation_from_gravity(finger_data->base.accel, finger_data->tip.accel);
	float weighted_curl = get_fusion_bendcurl(finger_data->base.accel, gyro_curl, accel_curl);
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
