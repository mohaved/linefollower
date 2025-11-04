#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include <stdint.h>

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
} PIDController;

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

// Line sensor data structure
typedef struct {
    uint16_t raw_values[8];
    uint16_t calibrated_min[8];
    uint16_t calibrated_max[8];
    float position;
    LinePattern pattern;
    uint8_t active_sensors;
} LineSensorArray;

// Function declarations

// Initialization functions
void init_line_follower(void);
void init_motors(void);
void init_sensors(void);
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

// Shared control structure for inter-core communication
typedef struct {
    mutex_t mutex;
    float pid_output;
    bool new_data;
} SharedControl;

extern SharedControl shared_control;

// Motor mutex to protect concurrent motor updates from both cores
extern mutex_t motor_mutex;

// Utility functions
void handle_intersection(LineSensorArray *sensors);

#endif // LINE_FOLLOWER_H