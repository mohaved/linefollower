#include "line_follower.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include <string.h>

// Global variables
static volatile RobotState current_state = STOPPED;
static LineSensorArray sensor_array;
static PIDController pid_controller;
static volatile bool core1_ready = false;
static absolute_time_t last_line_detected_time;
static int8_t last_direction = 0;

// Mutexes for synchronization between cores
static mutex_t sensor_mutex;
static mutex_t state_mutex;
static mutex_t _motor_mutex; // actual storage for motor mutex

// Shared control is declared in header; ensure we have storage here if not provided by main
// If main.c defines shared_control this weak definition will be ignored during linking.
SharedControl shared_control = { .mutex = {}, .pid_output = 0.0f, .new_data = false };

// Provide motor_mutex symbol
mutex_t motor_mutex;

// Initialize all components
void init_line_follower(void) {
    mutex_init(&sensor_mutex);
    mutex_init(&state_mutex);
    mutex_init(&motor_mutex);
    mutex_init(&shared_control.mutex);
    
    init_i2c();
    init_motors();
    init_sensors();
    init_buttons();
    
    // Initialize PID controller with tuned values
    init_pid_controller(&pid_controller, 1.0f, 0.001f, 2.0f);
    
    // Start core1
    multicore_launch_core1(core1_main);
    
    // Wait for core1 to be ready
    while (!core1_ready) {
        tight_loop_contents();
    }
}

// I2C initialization for ADS1115
void init_i2c(void) {
    // Initialize primary I2C (i2c0)
    i2c_init(i2c0, 400000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Initialize secondary I2C (i2c1) on separate pins to parallelize ADC reads
    // NOTE: Choose pins that don't conflict with motors/buttons. Using 8/9 by default.
    i2c_init(i2c1, 400000);
    gpio_set_function(I2C1_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA_PIN);
    gpio_pull_up(I2C1_SCL_PIN);
}

// Initialize motors with PWM
void init_motors(void) {
    // Configure PWM for all motor pins
    gpio_set_function(MOTOR_LEFT_PWM1, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_LEFT_PWM2, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_RIGHT_PWM1, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_RIGHT_PWM2, GPIO_FUNC_PWM);
    
    // Get PWM slices for each pin
    uint slice_left1 = pwm_gpio_to_slice_num(MOTOR_LEFT_PWM1);
    uint slice_left2 = pwm_gpio_to_slice_num(MOTOR_LEFT_PWM2);
    uint slice_right1 = pwm_gpio_to_slice_num(MOTOR_RIGHT_PWM1);
    uint slice_right2 = pwm_gpio_to_slice_num(MOTOR_RIGHT_PWM2);
    
    // Set PWM wrap value for all slices (255 for 8-bit resolution)
    pwm_set_wrap(slice_left1, 255);
    pwm_set_wrap(slice_left2, 255);
    pwm_set_wrap(slice_right1, 255);
    pwm_set_wrap(slice_right2, 255);
    
    // Enable all PWM slices
    pwm_set_enabled(slice_left1, true);
    pwm_set_enabled(slice_left2, true);
    pwm_set_enabled(slice_right1, true);
    pwm_set_enabled(slice_right2, true);
    
    // Initialize all PWM outputs to 0
    pwm_set_gpio_level(MOTOR_LEFT_PWM1, 0);
    pwm_set_gpio_level(MOTOR_LEFT_PWM2, 0);
    pwm_set_gpio_level(MOTOR_RIGHT_PWM1, 0);
    pwm_set_gpio_level(MOTOR_RIGHT_PWM2, 0);
}

// Initialize sensors and ADS1115
void init_sensors(void) {
    // Nothing to write globally here for single-shot mode.
    // We'll configure each channel on-demand in read_line_sensors().
}

// Initialize buttons with interrupt
void init_buttons(void) {
    gpio_init(CALIBRATION_BTN);
    gpio_init(START_STOP_BTN);
    
    gpio_set_dir(CALIBRATION_BTN, GPIO_IN);
    gpio_set_dir(START_STOP_BTN, GPIO_IN);
    
    gpio_pull_up(CALIBRATION_BTN);
    gpio_pull_up(START_STOP_BTN);
    
    gpio_set_irq_enabled_with_callback(CALIBRATION_BTN, GPIO_IRQ_EDGE_FALL, true, &button_callback);
    gpio_set_irq_enabled(START_STOP_BTN, GPIO_IRQ_EDGE_FALL, true);
}

// Button interrupt handler
void button_callback(uint gpio, uint32_t events) {
    if (gpio == CALIBRATION_BTN) {
        set_robot_state(CALIBRATING);
    } else if (gpio == START_STOP_BTN) {
        RobotState current = get_robot_state();
        set_robot_state(current == RUNNING ? STOPPED : RUNNING);
    }
}

// Read sensors through ADS1115
void read_line_sensors(LineSensorArray *sensors) {
    mutex_enter_blocking(&sensor_mutex);

    // To get lowest-latency sampling with two ADS1115 devices, we'll:
    // 1) Configure both ADS1115s for single-shot conversion on the same channel index
    // 2) Start conversions on both devices nearly simultaneously (write config to both i2c buses)
    // 3) Wait the required conversion time for the chosen data rate
    // 4) Read conversion registers from both devices
    // Repeat for channels 0..3. This effectively parallelizes ADC work and halves the total time vs
    // doing both devices sequentially on a single bus.

    // ADS1115 configuration constants for fastest sampling
    const uint16_t PGA_4_096 = (0x1 << 9); // ±4.096V
    const uint16_t MODE_SINGLE = (1 << 8);
    const uint16_t DR_860SPS = (0x7 << 5); // 860 samples per second (~1.16ms per conversion)

    // For channels 0..3: start both conversions then read
    for (int ch = 0; ch < 4; ch++) {
        // MUX for single-ended AINx: 100 + x (according to ADS1115 datasheet)
        uint16_t mux_left = (0x4 + (ch & 0x3)) << 12;
        uint16_t config_left = (1 << 15) | mux_left | PGA_4_096 | MODE_SINGLE | DR_860SPS;
        uint8_t cfg_left[3] = { ADS1115_REG_CONFIG, (uint8_t)(config_left >> 8), (uint8_t)(config_left & 0xFF) };

        uint16_t mux_right = (0x4 + (ch & 0x3)) << 12;
        uint16_t config_right = (1 << 15) | mux_right | PGA_4_096 | MODE_SINGLE | DR_860SPS;
        uint8_t cfg_right[3] = { ADS1115_REG_CONFIG, (uint8_t)(config_right >> 8), (uint8_t)(config_right & 0xFF) };

        // Start conversion on left ADS (i2c0) and right ADS (i2c1)
        i2c_write_blocking(i2c0, ADS1115_LEFT_ADDR, cfg_left, 3, false);
        i2c_write_blocking(i2c1, ADS1115_RIGHT_ADDR, cfg_right, 3, false);

        // Wait for conversion: at 860 SPS => ~1.16ms; use 2ms to be safe and allow I2C overhead
        sleep_ms(2);

        // Read left conversion
        uint8_t reg = ADS1115_REG_CONVERSION;
        i2c_write_blocking(i2c0, ADS1115_LEFT_ADDR, &reg, 1, false);
        uint8_t read_bytes_l[2];
        int rl = i2c_read_blocking(i2c0, ADS1115_LEFT_ADDR, read_bytes_l, 2, false);
        if (rl < 0) read_bytes_l[0] = read_bytes_l[1] = 0;
        int16_t signed_l = (int16_t)((read_bytes_l[0] << 8) | read_bytes_l[1]);
        if (signed_l < 0) signed_l = 0;
        sensors->raw_values[ch] = (uint16_t)signed_l;

        // Read right conversion
        i2c_write_blocking(i2c1, ADS1115_RIGHT_ADDR, &reg, 1, false);
        uint8_t read_bytes_r[2];
        int rr = i2c_read_blocking(i2c1, ADS1115_RIGHT_ADDR, read_bytes_r, 2, false);
        if (rr < 0) read_bytes_r[0] = read_bytes_r[1] = 0;
        int16_t signed_r = (int16_t)((read_bytes_r[0] << 8) | read_bytes_r[1]);
        if (signed_r < 0) signed_r = 0;
        sensors->raw_values[ch + 4] = (uint16_t)signed_r;
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
        
        if (value > 0.2f) { // Threshold for line detection
            weighted_sum += value * (i - 3.5f);
            sum += value;
            line_detected = true;
        }
    }
    
    if (!line_detected) {
        return 999.0f; // Line lost indicator
    }
    
    return weighted_sum / (sum * 3.5f); // Normalize to -1.0 to 1.0
}

// Handle line lost situation
void handle_line_lost(void) {
    absolute_time_t current_time = get_absolute_time();
    int64_t time_diff = absolute_time_diff_us(last_line_detected_time, current_time) / 1000;
    
    if (time_diff > 2000) { // If line lost for more than 2 seconds
        stop_motors();
        set_robot_state(STOPPED);
        return;
    }
    
    // Continue in last known direction
    if (last_direction < 0) {
        // Use motor mutex to avoid racing with core0 motor updates
        mutex_enter_blocking(&motor_mutex);
        set_motor_speeds(-100, 100); // Turn left
        mutex_exit(&motor_mutex);
    } else if (last_direction > 0) {
        mutex_enter_blocking(&motor_mutex);
        set_motor_speeds(100, -100); // Turn right
        mutex_exit(&motor_mutex);
    }
}

// Set motor speeds (-255 to 255)
void set_motor_speeds(int16_t left_speed, int16_t right_speed) {
    // Guard and clamp
    if (left_speed > 255) left_speed = 255;
    if (left_speed < -255) left_speed = -255;
    if (right_speed > 255) right_speed = 255;
    if (right_speed < -255) right_speed = -255;

    // Ensure single atomic update by caller using motor_mutex when needed.

    // Left motor
    if (left_speed >= 0) {
        pwm_set_gpio_level(MOTOR_LEFT_PWM1, (uint32_t)left_speed);
        pwm_set_gpio_level(MOTOR_LEFT_PWM2, 0);
    } else {
        pwm_set_gpio_level(MOTOR_LEFT_PWM1, 0);
        pwm_set_gpio_level(MOTOR_LEFT_PWM2, (uint32_t)(-left_speed));
    }
    
    // Right motor
    if (right_speed >= 0) {
        pwm_set_gpio_level(MOTOR_RIGHT_PWM1, (uint32_t)right_speed);
        pwm_set_gpio_level(MOTOR_RIGHT_PWM2, 0);
    } else {
        pwm_set_gpio_level(MOTOR_RIGHT_PWM1, 0);
        pwm_set_gpio_level(MOTOR_RIGHT_PWM2, (uint32_t)(-right_speed));
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
}

// Calculate PID output
float calculate_pid(PIDController *pid, float error) {
    pid->integral += error;
    float derivative = error - pid->previous_error;
    
    // Anti-windup
    if (pid->integral > 100.0f) pid->integral = 100.0f;
    if (pid->integral < -100.0f) pid->integral = -100.0f;
    
    float output = (pid->kp * error) + 
                  (pid->ki * pid->integral) + 
                  (pid->kd * derivative);
    
    pid->previous_error = error;
    return output;
}

// Core 1 main function
void core1_main(void) {
    core1_ready = true;
    
    while (1) {
        core1_sensor_handler();
        sleep_us(10); // Small delay to prevent overwhelming the system
    }
}

// Core 1 sensor handling
void core1_sensor_handler(void) {
    read_line_sensors(&sensor_array);
    float position = calculate_line_position(&sensor_array);
    
    if (position != 999.0f) {
        last_line_detected_time = get_absolute_time();
        last_direction = position > 0 ? 1 : -1;
        
        float pid_output = calculate_pid(&pid_controller, position);
        
        // Share PID output with Core 0
        mutex_enter_blocking(&shared_control.mutex);
        shared_control.pid_output = pid_output;
        shared_control.new_data = true;
        mutex_exit(&shared_control.mutex);
    } else {
        if (get_robot_state() == RUNNING) {
            handle_line_lost();
        }
    }
}

// Handle intersection detection
// Detect the current line pattern with detailed path analysis
LinePattern detect_line_pattern(LineSensorArray *sensors) {
    int active_sensors = 0;
    bool left_edge = false;    // Sensors 0-1
    bool left_mid = false;     // Sensors 2-3
    bool center = false;       // Sensors 3-4
    bool right_mid = false;    // Sensors 4-5
    bool right_edge = false;   // Sensors 6-7
    
    // Calculate threshold and check each sensor region
    for (int i = 0; i < 8; i++) {
        uint16_t threshold = (sensors->calibrated_max[i] + sensors->calibrated_min[i]) / 2;
        bool is_active = sensors->raw_values[i] > threshold;
        
        if (is_active) {
            active_sensors++;
            if (i < 2) left_edge = true;
            if (i >= 2 && i < 4) left_mid = true;
            if (i >= 3 && i < 5) center = true;
            if (i >= 4 && i < 6) right_mid = true;
            if (i >= 6) right_edge = true;
        }
    }
    
    sensors->active_sensors = active_sensors;
    
    // Advanced pattern detection logic
    if (active_sensors == 0) {
        return NO_LINE;
    }
    // Cross intersection (all regions active)
    else if (active_sensors >= 7 && left_edge && right_edge && center) {
        return CROSS_INTERSECTION;
    }
    // T intersection facing up (both sides and center active)
    else if (active_sensors >= 5 && left_edge && right_edge && center) {
        return T_INTERSECTION_UP;
    }
    // T intersection from left (right and center active, strong left signal)
    else if (active_sensors >= 4 && left_edge && center && !right_edge) {
        return T_INTERSECTION_LEFT;
    }
    // T intersection from right (left and center active, strong right signal)
    else if (active_sensors >= 4 && right_edge && center && !left_edge) {
        return T_INTERSECTION_RIGHT;
    }
    // Y intersection (gradual widening of active sensors)
    else if (active_sensors >= 5 && left_mid && right_mid && center) {
        return Y_INTERSECTION;
    }
    // 90-degree left turn
    else if (active_sensors >= 3 && left_edge && !right_edge && center) {
        return LEFT_TURN_90;
    }
    // 90-degree right turn
    else if (active_sensors >= 3 && right_edge && !left_edge && center) {
        return RIGHT_TURN_90;
    }
    // Line end detection
    else if (active_sensors <= 2 && (left_edge || right_edge) && !center) {
        return LINE_END;
    }
    
    return STRAIGHT_LINE;
}

// Handle different line patterns with middle path preference
void handle_intersection(LineSensorArray *sensors) {
    sensors->pattern = detect_line_pattern(sensors);
    
    switch (sensors->pattern) {
        case CROSS_INTERSECTION:
            // Always choose middle (straight) path at crossroads
            set_motor_speeds(200, 200);  // Full speed ahead
            while (sensors->active_sensors > 4) {  // Wait until we're past the intersection
                read_line_sensors(sensors);
                sleep_ms(1);
            }
            break;
            
        case T_INTERSECTION_UP:
            // At T junction, go straight (middle path)
            set_motor_speeds(200, 200);
            sleep_ms(200);  // Cross intersection quickly
            break;
            
        case T_INTERSECTION_LEFT:
        case T_INTERSECTION_RIGHT:
            // For side T-intersections, slow down and continue straight
            set_motor_speeds(150, 150);
            sleep_ms(150);
            break;
            
        case Y_INTERSECTION:
            // At Y splits, adjust to favor middle path
            // Slow down and use PID with center sensor bias
            set_motor_speeds(150, 150);
            while (sensors->active_sensors > 3) {
                float position = calculate_line_position(sensors);
                // Apply a center-seeking bias to the position
                position *= 0.7; // Reduce the impact of side sensors
                float pid_output = calculate_pid(&pid_controller, position);
                int16_t left_speed = 150 - (int16_t)pid_output;
                int16_t right_speed = 150 + (int16_t)pid_output;
                set_motor_speeds(left_speed, right_speed);
                read_line_sensors(sensors);
                sleep_ms(1);
            }
            break;
            
        case LEFT_TURN_90:
            if (!is_middle_path_available(sensors)) {
                // Only turn left if no middle path
                set_motor_speeds(-150, 150);
                while (!(sensors->raw_values[3] > sensors->calibrated_min[3] || 
                        sensors->raw_values[4] > sensors->calibrated_min[4])) {
                    read_line_sensors(sensors);
                    sleep_ms(1);
                }
            }
            break;
            
        case RIGHT_TURN_90:
            if (!is_middle_path_available(sensors)) {
                // Only turn right if no middle path
                set_motor_speeds(150, -150);
                while (!(sensors->raw_values[3] > sensors->calibrated_min[3] || 
                        sensors->raw_values[4] > sensors->calibrated_min[4])) {
                    read_line_sensors(sensors);
                    sleep_ms(1);
                }
            }
            break;
            
        case LINE_END:
            set_robot_state(STOPPED);
            break;
            
        case NO_LINE:
            handle_line_lost();
            break;
            
        case STRAIGHT_LINE:
            // Normal line following continues
            break;
    }
}

// Helper function to check if a middle path is available
bool is_middle_path_available(LineSensorArray *sensors) {
    // Check if center sensors detect a strong line
    return (sensors->raw_values[3] > (sensors->calibrated_max[3] + sensors->calibrated_min[3]) / 2 ||
            sensors->raw_values[4] > (sensors->calibrated_max[4] + sensors->calibrated_min[4]) / 2);
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