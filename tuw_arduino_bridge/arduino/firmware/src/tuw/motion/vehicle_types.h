#ifndef _VEHICLE_TYPES_H_
#define _VEHICLE_TYPES_H_

#ifdef VEHICLE_TYPE
	#if VEHICLE_TYPE == 1
		// Racecar
		#define ACKERMANN_CONFIG_L			0.155
		#define ACKERMANN_CONFIG_D			0.255
		#define ACKERMANN_CONFIG_GEAR_RATIO		10.0		/// @TODO Motor / Wheels
		#define ACKERMANN_CONFIG_WHEEL_RADIUS		0.0325
		#define ACKERMANN_CONFIG_MAX_STEER_ANGLE	(M_PI/8)
		#define ACKERMANN_CONFIG_WEIGHT			1		/// @TODO
		#define ACTUATOR_CONFIG_SERVO_DIRECTION		1
		#define ACTUATOR_CONFIG_SERVO_NEUTRAL		1600		/// Microseconds
		#define ACTUATOR_CONFIG_SERVO_US_PER_RAD	955		/// @TODO: copied from earlier version
		#define ACTUATOR_CONFIG_SERVO_RAD_MIN		-0.26		/// need not be symmetrical
		#define ACTUATOR_CONFIG_SERVO_RAD_MAX		0.27		/// just bounds in HW
	#elif VEHICLE_TYPE == 2
		// RocketCrawler
		#define ACKERMANN_CONFIG_L			0.188
		#define ACKERMANN_CONFIG_D			0.26
		#define ACKERMANN_CONFIG_GEAR_RATIO		122.0		/// Motor / Wheels
		#define ACKERMANN_CONFIG_WHEEL_RADIUS		0.05		// by calculation: 0.0517
		#define ACKERMANN_CONFIG_MAX_STEER_ANGLE	(M_PI/8)	/// @TODO
		#define ACKERMANN_CONFIG_WEIGHT			1		/// @TODO
		#define ACTUATOR_CONFIG_SERVO_DIRECTION		-1
		#define ACTUATOR_CONFIG_SERVO_NEUTRAL		1525		/// Microseconds
		#define ACTUATOR_CONFIG_SERVO_US_PER_RAD	1900
		#define ACTUATOR_CONFIG_SERVO_RAD_MIN		-0.226		/// need not be symmetrical
		#define ACTUATOR_CONFIG_SERVO_RAD_MAX		0.302		/// just bounds in HW
	#else
		#error Unknown VEHICLE_TYPE
	#endif
#else
	#error VEHICLE_TYPE not set
#endif


#endif
