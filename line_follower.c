#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "line_follower.h"

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

// Global variables
static volatile RobotState current_state = STOPPED;
static LineSensorArray sensor_array;
static PIDController pid_controller;
static absolute_time_t last_line_detected_time;
static int8_t last_direction = 0;
static float last_known_position = 0.0f;  // Last valid line position

// Mutexes for synchronization between cores
static mutex_t sensor_mutex;
static mutex_t state_mutex;
mutex_t motor_mutex;  // Defined here, declared extern in header

// Reference shared_control defined in main.c
extern SharedControl shared_control;

// Atomic flags for button events
volatile bool calibration_requested = false;
volatile bool start_stop_requested = false;
volatile bool core1_ready = false;

// Detect line pattern with improved accuracy and noise rejection
LinePattern detect_line_pattern(LineSensorArray *sensors) {
    int active_count = 0;
    bool left_edge = false;   // Sensors 0-1
    bool left_mid = false;    // Sensors 2-3
    bool center = false;      // Sensors 3-4
    bool right_mid = false;   // Sensors 4-5
    bool right_edge = false;  // Sensors 6-7
    
    // First normalize all sensor values (0.0-1.0)
    for (int i = 0; i < 8; i++) {
        uint16_t range = sensors->calibrated_max[i] - sensors->calibrated_min[i];
        if (range > 0) {
            sensors->normalized[i] = (float)(sensors->raw_values[i] - sensors->calibrated_min[i]) / range;
        } else {
            sensors->normalized[i] = 0.0f;
        }
        
        // Check if sensor is active using hysteresis
        if (sensors->normalized[i] > SENSOR_THRESHOLD) {
            active_count++;
            // Map sensor to region
            if (i < 2) left_edge = true;
            if (i >= 2 && i < 4) left_mid = true;
            if (i >= 3 && i < 5) center = true;
            if (i >= 4 && i < 6) right_mid = true;
            if (i >= 6) right_edge = true;
        }
    }
    
    sensors->active_sensors = active_count;
    
    // Pattern detection with noise rejection
    if (active_count == 0 || 
        (center && sensors->normalized[3] + sensors->normalized[4] < LINE_DETECT_MIN)) {
        return NO_LINE;
    }
    
    // Cross intersection - all regions active with strong center
    if (active_count >= CROSS_THRESHOLD && 
        left_edge && right_edge && center &&
        sensors->normalized[3] + sensors->normalized[4] > SENSOR_THRESHOLD * 2) {
        return CROSS_INTERSECTION;
    }
    
    // T intersection up - both edges and strong center
    if (active_count >= T_THRESHOLD && 
        left_edge && right_edge && center) {
        return T_INTERSECTION_UP;
    }
    
    // T intersection from sides
    if (active_count >= T_THRESHOLD && center) {
        if (left_edge && !right_edge) return T_INTERSECTION_LEFT;
        if (right_edge && !left_edge) return T_INTERSECTION_RIGHT;
    }
    
    // Y intersection - gradual widening with strong center
    if (active_count >= T_THRESHOLD && 
        left_mid && right_mid && center &&
        !left_edge && !right_edge) {
        return Y_INTERSECTION;
    }
    
    // 90-degree turns
    if (active_count >= TURN_THRESHOLD && center) {
        if (left_edge && !right_edge) return LEFT_TURN_90;
        if (right_edge && !left_edge) return RIGHT_TURN_90;
    }
    
    // Line end - few active sensors on one side only
    if (active_count <= LINE_END_THRESHOLD && !center) {
        if (left_edge || left_mid) return LINE_END;
        if (right_edge || right_mid) return LINE_END;
    }
    
    // Default: straight line or gentle curve
    return STRAIGHT_LINE;
}

// Initialize all components
void init_line_follower(void) {
    // Initialize all mutexes
    mutex_init(&sensor_mutex);
    mutex_init(&state_mutex);
    mutex_init(&motor_mutex);
    
    // Initialize shared control values
    mutex_enter_blocking(&shared_control.mutex);
    shared_control.base_speed = SPEED_CREEP;  // Start with slow speed
    shared_control.pid_output = 0.0f;
    shared_control.new_data = false;
    mutex_exit(&shared_control.mutex);
    
    init_multiplexer();
    init_motors();
    init_buttons();
    
    // Initialize PID controller with tuned values
    init_pid_controller(&pid_controller, KP, KI, KD);
    
    // No need to wait for core1 here - it will be launched by main()
}

// Initialize multiplexer for SVK IR sensors
void init_multiplexer(void) {
    // Initialize multiplexer select pins as outputs
    gpio_init(MUX_S0);
    gpio_init(MUX_S1);
    gpio_init(MUX_S2);

    gpio_set_dir(MUX_S0, GPIO_OUT);
    gpio_set_dir(MUX_S1, GPIO_OUT);
    gpio_set_dir(MUX_S2, GPIO_OUT);

    // Initialize ADC for reading sensor values
    adc_init();
    adc_gpio_init(MUX_SIG);
    adc_select_input(0);  // ADC0 (GPIO 26) is MUX_SIG
}

// Initialize motors for TB6612FNG
void init_motors(void) {
    // Configure direction pins as GPIO outputs
    gpio_init(MOTOR_LEFT_AIN1);
    gpio_init(MOTOR_LEFT_AIN2);
    gpio_init(MOTOR_RIGHT_BIN1);
    gpio_init(MOTOR_RIGHT_BIN2);
    gpio_init(MOTOR_STBY);

    gpio_set_dir(MOTOR_LEFT_AIN1, GPIO_OUT);
    gpio_set_dir(MOTOR_LEFT_AIN2, GPIO_OUT);
    gpio_set_dir(MOTOR_RIGHT_BIN1, GPIO_OUT);
    gpio_set_dir(MOTOR_RIGHT_BIN2, GPIO_OUT);
    gpio_set_dir(MOTOR_STBY, GPIO_OUT);

    // Configure PWM pins for speed control
    gpio_set_function(MOTOR_LEFT_PWMA, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_RIGHT_PWMB, GPIO_FUNC_PWM);

    // Get PWM slices for speed pins
    uint slice_left = pwm_gpio_to_slice_num(MOTOR_LEFT_PWMA);
    uint slice_right = pwm_gpio_to_slice_num(MOTOR_RIGHT_PWMB);

    // Set PWM wrap value (255 for 8-bit resolution)
    pwm_set_wrap(slice_left, 255);
    pwm_set_wrap(slice_right, 255);

    // Enable PWM slices
    pwm_set_enabled(slice_left, true);
    pwm_set_enabled(slice_right, true);

    // Initialize all outputs to 0 (motors stopped)
    gpio_put(MOTOR_LEFT_AIN1, 0);
    gpio_put(MOTOR_LEFT_AIN2, 0);
    gpio_put(MOTOR_RIGHT_BIN1, 0);
    gpio_put(MOTOR_RIGHT_BIN2, 0);
    gpio_put(MOTOR_STBY, 1);  // Enable motors (active high)

    pwm_set_gpio_level(MOTOR_LEFT_PWMA, 0);
    pwm_set_gpio_level(MOTOR_RIGHT_PWMB, 0);
}

// Initialize buttons with interrupt
void init_buttons(void) {
    gpio_init(CALIBRATION_BTN);
    gpio_init(START_STOP_BTN);
    
    gpio_set_dir(CALIBRATION_BTN, GPIO_IN);
    gpio_set_dir(START_STOP_BTN, GPIO_IN);
    
    gpio_pull_up(CALIBRATION_BTN);
    gpio_pull_up(START_STOP_BTN);
    
    // Use the same callback for both buttons
    gpio_set_irq_enabled_with_callback(CALIBRATION_BTN, GPIO_IRQ_EDGE_FALL, true, &button_callback);
    gpio_set_irq_enabled_with_callback(START_STOP_BTN, GPIO_IRQ_EDGE_FALL, true, &button_callback);
}

// Button interrupt handler - just sets flags
void button_callback(uint gpio, uint32_t events) {
    if (gpio == CALIBRATION_BTN) {
        calibration_requested = true;
    } else if (gpio == START_STOP_BTN) {
        start_stop_requested = true;
    }
}

// Read sensors through multiplexer and ADC
void read_line_sensors(LineSensorArray *sensors) {
    mutex_enter_blocking(&sensor_mutex);

    for (int i = 0; i < 8; i++) {
        // Select multiplexer channel (S0, S1, S2)
        gpio_put(MUX_S0, i & 0x01);
        gpio_put(MUX_S1, (i >> 1) & 0x01);
        gpio_put(MUX_S2, (i >> 2) & 0x01);

        // Small delay for multiplexer settling
        sleep_us(10);

        // Read ADC value (12-bit, 0-4095)
        sensors->raw_values[i] = adc_read();
    }

    mutex_exit(&sensor_mutex);
}

// Calibrate sensors
void calibrate_sensors(LineSensorArray *sensors) {
    for (int i = 0; i < 8; i++) {
        sensors->calibrated_min[i] = UINT16_MAX;
        sensors->calibrated_max[i] = 0;
    }
    
    // Calibrate for 5 seconds
    absolute_time_t end_time = make_timeout_time_ms(5000);
    while (!time_reached(end_time)) {
        read_line_sensors(sensors);
        
        for (int i = 0; i < 8; i++) {
            if (sensors->raw_values[i] < sensors->calibrated_min[i])
                sensors->calibrated_min[i] = sensors->raw_values[i];
            if (sensors->raw_values[i] > sensors->calibrated_max[i])
                sensors->calibrated_max[i] = sensors->raw_values[i];
        }
    }
}

// Calculate line position (-1.0 to 1.0)
float calculate_line_position(LineSensorArray *sensors) {
    float weighted_sum = 0;
    float sum = 0;
    bool line_detected = false;
    
    for (int i = 0; i < 8; i++) {
        uint16_t denom = sensors->calibrated_max[i] - sensors->calibrated_min[i];
        float value = 0.0f;
        if (denom > 0) {
            value = (float)(sensors->raw_values[i] - sensors->calibrated_min[i]) / (float)denom;
        }
        
        if (value > LINE_DETECT_THRESH) { // Threshold for line detection
            weighted_sum += value * (i - 3.5f);
            sum += value;
            line_detected = true;
        }
    }
    
    if (!line_detected) {
        return 999.0f; // Line lost indicator
    }
    
    return weighted_sum / (sum * LINE_POSITION_NORM); // Normalize to -1.0 to 1.0
}

// Smart line recovery with systematic search pattern
void handle_line_lost(void) {
    absolute_time_t current_time = get_absolute_time();
    int64_t time_diff = absolute_time_diff_us(last_line_detected_time, current_time) / 1000;
    static float search_angle = 0;
    static bool increasing_angle = true;
    static bool first_search = true;

    printf("DEBUG: Line lost for %lld ms\n", time_diff);

    if (time_diff > MAX_SEARCH_TIME_MS) { // Extended search time - 10 seconds
        printf("DEBUG: No line found for 10 seconds, stopping robot\n");
        stop_motors();
        set_robot_state(STOPPED);
        // Reset search pattern for next time
        search_angle = 0;
        increasing_angle = true;
        first_search = true;
        return;
    }

    // Initialize search from last known position
    if (first_search) {
        search_angle = last_known_position * 15.0f; // Convert position (-1..1) to initial angle
        first_search = false;
    }

    // Systematic search pattern
    mutex_enter_blocking(&motor_mutex);
    
    // Calculate turn speeds based on current search angle
    float turn_intensity = fabsf(search_angle) / SEARCH_ANGLE_MAX;
    int16_t inner_speed = (int16_t)(SPEED_CREEP * (1.0f - turn_intensity));
    int16_t outer_speed = (int16_t)(SPEED_CREEP);

    if (search_angle < 0) {
        // Search left
        set_motor_speeds(-inner_speed, outer_speed);
    } else {
        // Search right
        set_motor_speeds(outer_speed, -inner_speed);
    }
    mutex_exit(&motor_mutex);

    // Update search pattern
    if (increasing_angle) {
        search_angle += SEARCH_ANGLE_STEP; // Increment by defined step
        if (search_angle >= SEARCH_ANGLE_MAX) {
            increasing_angle = false;
        }
    } else {
        search_angle -= SEARCH_ANGLE_STEP; // Decrement by defined step
        if (search_angle <= -SEARCH_ANGLE_MAX) {
            increasing_angle = true;
        }
    }

    // Small delay to allow smooth movement
    sleep_ms(10);
}

// Set motor speeds (-255 to 255) for TB6612FNG
void set_motor_speeds(int16_t left_speed, int16_t right_speed) {
    // Guard and clamp
    if (left_speed > 255) left_speed = 255;
    if (left_speed < -255) left_speed = -255;
    if (right_speed > 255) right_speed = 255;
    if (right_speed < -255) right_speed = -255;

    // Ensure single atomic update by caller using motor_mutex when needed.

    // Left motor control
    if (left_speed > 0) {
        // Forward: AIN1=1, AIN2=0, PWMA=speed
        gpio_put(MOTOR_LEFT_AIN1, 1);
        gpio_put(MOTOR_LEFT_AIN2, 0);
        pwm_set_gpio_level(MOTOR_LEFT_PWMA, (uint16_t)left_speed);
    } else if (left_speed < 0) {
        // Backward: AIN1=0, AIN2=1, PWMA=speed
        gpio_put(MOTOR_LEFT_AIN1, 0);
        gpio_put(MOTOR_LEFT_AIN2, 1);
        pwm_set_gpio_level(MOTOR_LEFT_PWMA, (uint16_t)(-left_speed));
    } else {
        // Stop: AIN1=0, AIN2=0, PWMA=0
        gpio_put(MOTOR_LEFT_AIN1, 0);
        gpio_put(MOTOR_LEFT_AIN2, 0);
        pwm_set_gpio_level(MOTOR_LEFT_PWMA, 0);
    }

    // Right motor control
    if (right_speed > 0) {
        // Forward: BIN1=1, BIN2=0, PWMB=speed
        gpio_put(MOTOR_RIGHT_BIN1, 1);
        gpio_put(MOTOR_RIGHT_BIN2, 0);
        pwm_set_gpio_level(MOTOR_RIGHT_PWMB, (uint16_t)right_speed);
    } else if (right_speed < 0) {
        // Backward: BIN1=0, BIN2=1, PWMB=speed
        gpio_put(MOTOR_RIGHT_BIN1, 0);
        gpio_put(MOTOR_RIGHT_BIN2, 1);
        pwm_set_gpio_level(MOTOR_RIGHT_PWMB, (uint16_t)(-right_speed));
    } else {
        // Stop: BIN1=0, BIN2=0, PWMB=0
        gpio_put(MOTOR_RIGHT_BIN1, 0);
        gpio_put(MOTOR_RIGHT_BIN2, 0);
        pwm_set_gpio_level(MOTOR_RIGHT_PWMB, 0);
    }
}

// Stop both motors
void stop_motors(void) {
    set_motor_speeds(0, 0);
}

// Initialize PID controller
void init_pid_controller(PIDController *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->previous_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
}

// High-performance PID controller with advanced features
float calculate_pid(PIDController *pid, float error) {
    // Deadband to prevent oscillation on small errors
    const float DEAD_BAND = 0.02f;
    if (fabsf(error) < DEAD_BAND) {
        error = 0.0f;
    }

    // Proportional term
    float p_term = pid->kp * error;

    // Derivative term with filtering to reduce noise
    float derivative = error - pid->previous_error;
    const float DERIVATIVE_FILTER = 0.8f;  // Low-pass filter coefficient
    derivative = DERIVATIVE_FILTER * derivative + (1.0f - DERIVATIVE_FILTER) * pid->derivative;
    pid->derivative = derivative;
    float d_term = pid->kd * derivative;

    // Integral term with advanced anti-windup
    float potential_output = p_term + d_term;
    const float OUTPUT_LIMIT = 95.0f;  // Slightly less than max to allow headroom

    // Only integrate if output won't be saturated (back-calculation anti-windup)
    if (fabsf(potential_output) < OUTPUT_LIMIT) {
        pid->integral += pid->ki * error;
    } else {
        // Back-calculate to prevent windup
        float back_calc = (potential_output > 0 ? OUTPUT_LIMIT - potential_output : -OUTPUT_LIMIT - potential_output);
        pid->integral += pid->ki * back_calc;
    }

    // Clamp integral to prevent excessive accumulation
    const float INTEGRAL_LIMIT = 80.0f;
    if (pid->integral > INTEGRAL_LIMIT) pid->integral = INTEGRAL_LIMIT;
    if (pid->integral < -INTEGRAL_LIMIT) pid->integral = -INTEGRAL_LIMIT;

    // Calculate final output
    float output = p_term + pid->integral + d_term;

    // Output limiting
    if (output > OUTPUT_LIMIT) output = OUTPUT_LIMIT;
    if (output < -OUTPUT_LIMIT) output = -OUTPUT_LIMIT;

    // Store error for next iteration
    pid->previous_error = error;

    return output;
}

// High-performance Core 1 main function with optimized timing
void core1_main(void) {
    core1_ready = true;

    // Set up high-priority execution for real-time sensor processing
    // Use tight timing loop for maximum responsiveness
    absolute_time_t next_update = get_absolute_time();

    while (1) {
        // Execute sensor handler at fixed high frequency (2kHz for optimal performance)
        core1_sensor_handler();

        // Fixed update rate for consistent timing (500us = 2kHz)
        // This provides optimal balance between responsiveness and CPU usage
        next_update = delayed_by_us(next_update, 500);
        sleep_until(next_update);
    }
}

// Core 1 sensor handling
void core1_sensor_handler(void) {
    // Read and analyze sensor data
    read_line_sensors(&sensor_array);
    float position = calculate_line_position(&sensor_array);
    LinePattern pattern = detect_line_pattern(&sensor_array);
    sensor_array.pattern = pattern;  // Store for debugging/telemetry
    
    if (position != 999.0f) {
        last_line_detected_time = get_absolute_time();
        last_known_position = position;
        
        // Base speed selection based on pattern and position
        uint16_t base_speed;
        float position_abs = fabsf(position);
        
        switch (pattern) {
            case CROSS_INTERSECTION:
            case T_INTERSECTION_UP:
                // Maintain speed through straight intersections
                base_speed = SPEED_STRAIGHT;
                // Center bias for PID
                position *= 0.7f;
                break;
                
            case T_INTERSECTION_LEFT:
            case T_INTERSECTION_RIGHT:
            case Y_INTERSECTION:
                // Slow down for side intersections
                base_speed = SPEED_TURN;
                break;
                
            case LEFT_TURN_90:
            case RIGHT_TURN_90:
                // Very slow for sharp turns
                base_speed = SPEED_CREEP;
                break;
                
            case LINE_END:
                // Stop at end
                stop_motors();
                set_robot_state(STOPPED);
                return;
                
            default:  // STRAIGHT_LINE or gentle curves
                if (position_abs < 0.2f) {
                    base_speed = SPEED_STRAIGHT;
                } else if (position_abs < 0.5f) {
                    base_speed = SPEED_CURVE;
                } else {
                    base_speed = SPEED_TURN;
                }
                break;
        }
        
        // Calculate PID with adjusted position
        float pid_output = calculate_pid(&pid_controller, position);
        
        // Share data with Core 0
        mutex_enter_blocking(&shared_control.mutex);
        shared_control.pid_output = pid_output;
        shared_control.base_speed = base_speed;
        shared_control.new_data = true;
        mutex_exit(&shared_control.mutex);
    } else {
        set_robot_state(LINE_LOST);
        handle_line_lost();
    }
}

// Set robot state with mutex protection
void set_robot_state(RobotState new_state) {
    mutex_enter_blocking(&state_mutex);
    current_state = new_state;
    
    if (new_state == CALIBRATING) {
        calibrate_sensors(&sensor_array);
        current_state = STOPPED;
    } else if (new_state == STOPPED) {
        stop_motors();
    }
    
    mutex_exit(&state_mutex);
}

// Get robot state with mutex protection
RobotState get_robot_state(void) {
    mutex_enter_blocking(&state_mutex);
    RobotState state = current_state;
    mutex_exit(&state_mutex);
    return state;
}