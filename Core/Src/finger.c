#include <math.h>
#include "finger.h"

float accel_relative_rotation_from_gravity(vec3 lower_gravity_vector, vec3 upper_gravity_vector){
    L2_vec_norm(&lower_gravity_vector);
    L2_vec_norm(&upper_gravity_vector);
    // # Step 3: Calculate Rotation Angle
	float mag = magnitude(lower_gravity_vector);
    float cos = dot_vec3(lower_gravity_vector, upper_gravity_vector);
    return (180.0/M_PI)*((float)acos(cos));
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
    float R = (10*PEAK_CONST)/((mag-g)*(mag-g) + PEAK_CONST);
    float mag_gyro_weight = Q/(R+Q);
    float mag_accel_weight = R/(R+Q);

    // perform orientation-based weighting
    L2_vec_norm(&gravity_vector);
    float cos = gravity_vector.x; //weight is the cos of the angle between (1 if gravity aligned with x axis, 0 if orthogonal to x-axis)
    float orthog_accel_weight = 1-cos;//sqrt(1 - cos*cos); // get the sine of the angle cos^2 = 1 - sin^2
//    PDEBUG("cos: %f\n", cos);
    if (0.8 < fabs(cos)){
    	orthog_accel_weight = 0;
    }

    float accel_weight = orthog_accel_weight*mag_accel_weight;
    float gyro_weight = 1 - accel_weight;
    float weighted_angle = gyro_weight*gyro_bendcurl + accel_weight*accel_bendcurl;
    return weighted_angle;//accel_weight*accel_bendcurl + gyro_weight*gyro_bendcurl;
}

float get_fusion_wag(vec3 gravity_vector, float gyro_wag, float accel_wag){
    // perform magnitude-based weighting
    float mag = magnitude(gravity_vector);
    float R = (10*PEAK_CONST)/((mag-g)*(mag-g) + PEAK_CONST);
    float mag_gyro_weight = Q/(R+Q);
    float mag_accel_weight = R/(R+Q);

    // perform orientation-based weighting
    L2_vec_norm(&gravity_vector);
    float cos = fabs(gravity_vector.x); //weight is the sin of the angle between (1 if gravity aligned with x axis, 0 if orthogonal to x-axis)
    float orthog_accel_weight = 1-sqrt(1-cos*cos);//sqrt(1 - cos*cos); // get the sine of the angle cos^2 = 1 - sin^2
//    PDEBUG("cos: %f\n", cos);

    float accel_weight = orthog_accel_weight*mag_accel_weight;
    float gyro_weight = 1 - accel_weight;
    float weighted_angle = gyro_weight*gyro_wag + accel_weight*accel_wag;
    return weighted_angle;//accel_weight*accel_bendcurl + gyro_weight*gyro_bendcurl;
}

float get_bend(IMUData* hand_data, IMUData* base_data, int16_t frequency){
	float gyro_bend = (base_data->gyro.roll - hand_data->gyro.roll)/frequency;
    return gyro_bend;
}

float get_curl(FingerSensorData* finger_data, int16_t frequency) {
	float gyro_curl = (finger_data->tip.gyro.roll - finger_data->base.gyro.roll)/frequency;
    return gyro_curl;
}

float get_wag(IMUData* hand_data, FingerSensorData* finger_data, int16_t frequency) {
	float avg_finger_gyro_yaw = (finger_data->base.gyro.yaw + finger_data->tip.gyro.yaw)/2;
	float gyro_wag = (finger_data->base.gyro.yaw - hand_data->gyro.yaw)/frequency;
	return gyro_wag;
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
//    PDEBUG("bend change: %f\n", bend_change);
    float gyro_bend = fmod(finger->bend, 360);

//    PDEBUG("mag: %f\n", magnitude(finger_data->base.accel));

    // update curl
    float curl_change = get_curl(finger_data, frequency);
    if (fabs(curl_change) < 150) finger->curl += curl_change;
//    PDEBUG("curl change: %f\n", curl_change);
    float gyro_curl = fmod(finger->curl, 360);


    float accel_bend = accel_relative_rotation_from_gravity(hand_data->accel, finger_data->base.accel);
	float accel_curl = accel_relative_rotation_from_gravity(finger_data->base.accel, finger_data->tip.accel);

	float fusion_bend = get_fusion_bendcurl(finger_data->base.accel, gyro_bend, accel_bend);
	float fusion_curl = get_fusion_bendcurl(finger_data->base.accel, gyro_curl, accel_curl);

	finger->bend /= 2;
	finger->curl /= 2;
	finger->bend += fusion_bend / 2;
	finger->curl += fusion_curl / 2;

    // update wag
	float fusion_wag;
	if (finger->bend < 30) {
	    float wag_change = get_wag(hand_data, finger_data, frequency);
	    float pitch_change = (finger_data->base.gyro.pitch + hand_data->gyro.pitch)/2;
	    if (fabs(pitch_change) < fabs(wag_change)*4) finger->wag += wag_change;//fusion_wag = finger->wag + wag_change;
	    float gyro_wag = fmod(finger->wag, 360);
//		float accel_wag = accel_bend + 10; // calculated the same for the accelerometers
//		fusion_wag = get_fusion_wag(hand_data->accel, gyro_wag, accel_wag);
		fusion_wag = gyro_wag;

		int wag_bound = 30;
		if (fusion_wag < -wag_bound) fusion_wag = -wag_bound;
		else if (fusion_wag > wag_bound) fusion_wag = wag_bound;
	}
	else // can't wag with a clenched fist
		finger->wag /= 2;
//	finger->wag /= 2;
//	finger->wag += fusion_wag / 2;

}

void calibrate_thumb(Finger* thumb) {
    calibrate_finger(thumb);
}

void update_thumb(Finger* thumb, FingerSensorData* finger_data, FingerSensorData* index_data, int16_t frequency, float flex_data[]) {
    // could probably do some sensor fusion here to make the finger data more accurate

	// update bend (using palm flex sensor)
	float palm_flex_bend = flex_data[0];
	int palm_bias = 15;
	float palm_bend = 180 - palm_flex_bend - palm_bias;
	thumb->bend = thumb->bend/2 + palm_bend/2;

	// update curl (with IMUs)
	float gyro_curl_change = get_curl(finger_data, frequency);
    if (fabs(gyro_curl_change) < 150) thumb->curl += gyro_curl_change;
    float gyro_curl = fmod(thumb->curl, 360);
	float accel_curl = accel_relative_rotation_from_gravity(finger_data->base.accel, finger_data->tip.accel);
	float fusion_curl = get_fusion_bendcurl(finger_data->base.accel, gyro_curl, accel_curl);
	thumb->curl = thumb->curl/2 + fusion_curl/2;

	// update wag angle (with index imu data and maybe flex sensor)
	float avg_index_gyro_yaw = (index_data->base.gyro.yaw + index_data->tip.gyro.yaw)/2;
	float avg_thumb_gyro_yaw = (finger_data->base.gyro.yaw + finger_data->tip.gyro.yaw)/2;
	float gyro_wag = (avg_thumb_gyro_yaw - avg_index_gyro_yaw)/frequency;
	thumb->wag += gyro_wag;
//    float accel_wag = accel_relative_rotation_from_gravity(hand_data->accel, finger_data->base.accel);

}

void initialize_finger(Finger *finger,
		FingerSensorData *finger_sensor_data
		) {
	*finger = (Finger) {
		.curl = 0,
		.bend = 0,
		.wag = 0
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
