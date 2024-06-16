/*
 * globals.cpp
 *
 *  Created on: 25.09.2017
 *      Author: anonymous
 */

#include <Arduino.h>
#include "defines.h"

String MAC;  // MAC address of Wifi module

hw_timer_t * timer1 = NULL;
hw_timer_t * timer2 = NULL;

uint8_t cascade_control_loop_counter = 0;
uint8_t loop_counter;       // To generate a medium loop 40Hz
uint8_t slow_loop_counter;  // slow loop 2Hz
uint8_t sendBattery_counter; // To send battery status
//int16_t BatteryValue;

// Battery Defines
float batteryVolts = 12;     // Estimated Robot Battery voltage
float resUpper =     39000;  // Upper resistor value of battery voltage divider 
float resLower =     10000;  // Lower resistor value of battery voltage divider
float voltRef =      3.3;    // AtoD Ref for ESP32
float ADrange =      4095;   // AtoD Range of ESP32

long timer_old;
long timer_value;
float debugVariable;
float dt;

// Angle of the robot (used for stability control)
float angle_adjusted;
float angle_adjusted_Old;
float angle_adjusted_filtered=0.0;

// Default control values from constant definitions
float Kp = KP;
float Kd = KD;
float Kp_thr = KP_THROTTLE;
float Ki_thr = KI_THROTTLE;
float Kp_user = KP;
float Kd_user = KD;
float Kp_thr_user = KP_THROTTLE;
float Ki_thr_user = KI_THROTTLE;
float Kp_position = KP_POSITION;
float Kd_position = KD_POSITION;
float LeftShoulder;
float JoyStickLftRgt;
float JoyStickForRev;
float Kp_user_Preset;
float Kd_user_Preset;
float Kp_thr_user_Preset;
float Ki_thr_user_Preset;
bool newControlParameters = false;
bool modifing_control_parameters = false;
int16_t position_error_sum_M1;
int16_t position_error_sum_M2;
float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;
float target_angle;
int16_t throttle;
float steering;
float max_throttle = MAX_THROTTLE;
float max_steering = MAX_STEERING;
float max_target_angle = MAX_TARGET_ANGLE;
float control_output;
float angle_offset = ANGLE_OFFSET;

boolean positionControlMode = false;

int16_t motor1;
int16_t motor2;

// position control
volatile int32_t steps1;
volatile int32_t steps2;
int32_t target_steps1;
int32_t target_steps2;
int16_t motor1_control;
int16_t motor2_control;

int16_t speed_M1, speed_M2;        // Actual speed of motors
int8_t  dir_M1, dir_M2;            // Actual direction of steppers motors
int16_t actual_robot_speed;        // overall robot speed (measured from steppers speed)
int16_t actual_robot_speed_Old;
float estimated_speed_filtered;    // Estimated robot speed
