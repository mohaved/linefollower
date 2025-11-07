#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include <stdint.h>

// PID and Control Constants
#define PID_SCALE           100.0f    // PID output scaling factor
#define TURN_ASYMMETRY      1.2f      // Inner wheel reduction factor for turns
#define LINE_DETECT_THRESH  0.2f      // Threshold for line detection
#define LINE_POSITION_NORM  3.5f      // Position normalization factor
#define MAX_SEARCH_TIME_MS  10000     // Maximum line search time
#define SEARCH_ANGLE_STEP   5.0f      // Search pattern angle increment
#define PWM_MAX            255        // Maximum PWM value

// Pattern detection thresholds
#define SENSOR_THRESHOLD     0.5f    // Threshold for sensor activation (normalized)
#define LINE_DETECT_MIN      0.2f    // Minimum value to consider line detected
#define CROSS_THRESHOLD      7       // Minimum sensors for cross intersection
#define T_THRESHOLD         5       // Minimum sensors for T intersection
#define TURN_THRESHOLD      3       // Minimum sensors for 90-degree turn
#define LINE_END_THRESHOLD  2       // Maximum sensors for line end

// Speed profiles for different path types
#define SPEED_STRAIGHT    255  // Maximum speed on straight paths
#define SPEED_CURVE      180  // Medium speed for curves
#define SPEED_TURN       150  // Slower for sharp turns
#define SPEED_CREEP      100  // Very slow for precise movements
#define SEARCH_ANGLE_MAX  45  // Maximum search angle (degrees)
#define MIN_SPEED_HIGH     60         // Minimum speed for high base_speed
#define MIN_SPEED_LOW      40         // Minimum speed for low base_speed
#define HIGH_SPEED_THRESH  180        // Threshold for high/low speed behavior

// Pin definitions
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1
// Optional second I2C bus (useful to parallelize ADS1115 reads)
#define I2C1_SDA_PIN 8
#define I2C1_SCL_PIN 9

// Motor control PWM pins for TC1508A
#define MOTOR_LEFT_PWM1  2  // Left motor forward
#define MOTOR_LEFT_PWM2  3  // Left motor backward
#define MOTOR_RIGHT_PWM1 4  // Right motor forward
#define MOTOR_RIGHT_PWM2 5  // Right motor backward

// Button pins
#define CALIBRATION_BTN 14
#define START_STOP_BTN  15

// ADS1115 settings
#define ADS1115_LEFT_ADDR  0x48  // ADDR -> GND (1001000)
#define ADS1115_RIGHT_ADDR 0x49  // ADDR -> VDD (1001001)
#define ADS1115_REG_CONVERSION 0x00
#define ADS1115_REG_CONFIG 0x01

#define KP 1.0f
#define KI 0.001f
#define KD 2.0f

// Line pattern types
typedef enum {
    STRAIGHT_LINE,
    LEFT_TURN_90,
    RIGHT_TURN_90,
    T_INTERSECTION_LEFT,   // ├
    T_INTERSECTION_RIGHT,  // ┤
    T_INTERSECTION_UP,     // ┴
    CROSS_INTERSECTION,    // ┼
    Y_INTERSECTION,        // Y split
    LINE_END,
    NO_LINE
} LinePattern;

// Robot states
typedef enum {
    STOPPED,
    RUNNING,
    CALIBRATING,
    LINE_LOST
} RobotState;

// PID parameters structure
typedef struct {
    float kp;
    float ki;
    float kd;
    float previous_error;
    float integral;
    float derivative;
} PIDController;

// Intersection action types
typedef enum {
    NO_ACTION,
    FORWARD_CROSS,     // Go straight through intersection
    SLOW_FORWARD,      // Slow down and continue straight
    TURN_LEFT,         // Make a left turn
    TURN_RIGHT,        // Make a right turn
    Y_SPLIT_CENTER     // Handle Y intersection with center bias
} IntersectionAction;

// Shared control structure for inter-core communication
typedef struct {
    mutex_t mutex;
    float pid_output;
    uint16_t base_speed;      // Dynamic base speed based on path type
    bool new_data;
    IntersectionAction intersection_action;  // Current intersection handling request
    bool intersection_active;                // True while handling an intersection
    LinePattern current_pattern;            // Current detected line pattern
} SharedControl;

// Line sensor data structure
typedef struct {
    uint16_t raw_values[8];
    uint16_t calibrated_min[8];
    uint16_t calibrated_max[8];
    float position;
    LinePattern pattern;    // Current detected pattern
    uint8_t active_sensors; // Number of active sensors
    float normalized[8];    // Normalized sensor values (0.0-1.0)
} LineSensorArray;

// Initialization functions
void init_line_follower(void);
void init_motors(void);
void init_i2c(void);
void init_buttons(void);

// Sensor functions
void read_line_sensors(LineSensorArray *sensors);
void calibrate_sensors(LineSensorArray *sensors);
float calculate_line_position(LineSensorArray *sensors);
void handle_line_lost(void);

// Motor control functions
void set_motor_speeds(int16_t left_speed, int16_t right_speed);
void stop_motors(void);

// PID control functions
void init_pid_controller(PIDController *pid, float kp, float ki, float kd);
float calculate_pid(PIDController *pid, float error);

// Button interrupt handlers
void button_callback(uint gpio, uint32_t events);

// Core 1 functions
void core1_main(void);
void core1_sensor_handler(void);

// State management
void set_robot_state(RobotState new_state);
RobotState get_robot_state(void);

extern SharedControl shared_control;

// Motor mutex to protect concurrent motor updates from both cores
extern mutex_t motor_mutex;

// Utility functions
LinePattern detect_line_pattern(LineSensorArray *sensors);

#endif // LINE_FOLLOWER_H