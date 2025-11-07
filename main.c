#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/mutex.h"
#include "line_follower.h"

// External button flags from line_follower.c
extern volatile bool calibration_requested;
extern volatile bool start_stop_requested;
extern volatile bool core1_ready;

// Helper function to constrain values within a range
static inline int16_t constrain(int16_t val, int16_t min, int16_t max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

// Shared data structure between cores
// Define shared_control instance declared in header
SharedControl shared_control = { .mutex = {}, .pid_output = 0.0f, .new_data = false };

// High-performance Core 0: Optimized motor control with real-time processing
void core0_motor_control() {
    // Initialize timing for consistent 2kHz update rate
    absolute_time_t next_update = get_absolute_time();

    while (1) {
        mutex_enter_blocking(&shared_control.mutex);
        bool has_new_data = shared_control.new_data;
        float pid_output = shared_control.pid_output;
        uint16_t base_speed = shared_control.base_speed;
        shared_control.new_data = false;
        mutex_exit(&shared_control.mutex);

        if (has_new_data && get_robot_state() == RUNNING) {
            // Scale PID correction based on current base_speed
            float correction_scale = (float)base_speed / PWM_MAX; // Normalize to max speed
            int16_t correction = (int16_t)(pid_output * PID_SCALE * correction_scale);

            // Apply asymmetric correction for better turning
            int16_t left_speed, right_speed;
            if (correction > 0) {
                // Turning right - slow down right motor more
                left_speed = base_speed;
                right_speed = base_speed - (correction * TURN_ASYMMETRY); // 20% stronger inner wheel reduction
            } else {
                // Turning left - slow down left motor more
                left_speed = base_speed + (correction * TURN_ASYMMETRY); // 20% stronger inner wheel reduction
                right_speed = base_speed;
            }
            // Dynamic minimum speed to prevent stalling
            int16_t min_speed = base_speed > HIGH_SPEED_THRESH ? MIN_SPEED_HIGH : MIN_SPEED_LOW;

            // Constrain motor speeds with dynamic minimum
            left_speed = constrain(left_speed, min_speed, PWM_MAX);
            right_speed = constrain(right_speed, min_speed, PWM_MAX);

            // Add acceleration limiting
            static int16_t prev_left_speed = 0;
            static int16_t prev_right_speed = 0;
            const int16_t max_accel = 20; // Maximum speed change per iteration

            // Limit acceleration
            left_speed = constrain(left_speed,
                prev_left_speed - max_accel,
                prev_left_speed + max_accel);
            right_speed = constrain(right_speed,
                prev_right_speed - max_accel,
                prev_right_speed + max_accel);

            // Store current speeds for next iteration
            prev_left_speed = left_speed;
            prev_right_speed = right_speed;

            // Clamp and apply motor speeds inside motor mutex
            mutex_enter_blocking(&motor_mutex);
            set_motor_speeds(left_speed, right_speed);
            mutex_exit(&motor_mutex);
        }

        // Fixed 2kHz update rate for maximum responsiveness and synchronization with Core 1
        next_update = delayed_by_us(next_update, 500);
        sleep_until(next_update);
    }
}

int main() {
    // Initialize stdio for debug output
    stdio_init_all();

    // Initialize mutex
    mutex_init(&shared_control.mutex);

    // Initialize the line follower system
    init_line_follower();

    printf("DEBUG: Line follower robot initialized\n");
    printf("DEBUG: Press CALIBRATION button to calibrate sensors\n");
    printf("DEBUG: Press START/STOP button to start/stop the robot\n");

    // Start core1 for sensor processing
    multicore_launch_core1(core1_main);

    // Wait for core1 to be ready
    while (!core1_ready) {
        sleep_ms(10);
    }

    printf("DEBUG: Core 1 ready, starting main control loop\n");

    // Main control loop - handle button events and state management
    while (1) {
        // Check for calibration button press
        if (calibration_requested) {
            calibration_requested = false; // Clear flag
            printf("DEBUG: Calibration button pressed - starting sensor calibration\n");
            set_robot_state(CALIBRATING);
            printf("DEBUG: Calibration complete\n");
        }

        // Check for start/stop button press
        if (start_stop_requested) {
            start_stop_requested = false; // Clear flag
            RobotState current_state = get_robot_state();
            if (current_state == RUNNING) {
                printf("DEBUG: Stop button pressed - stopping robot\n");
                set_robot_state(STOPPED);
            } else if (current_state == STOPPED) {
                printf("DEBUG: Start button pressed - starting robot\n");
                set_robot_state(RUNNING);
            }
        }

        // Small delay to prevent busy waiting
        sleep_ms(10);
    }

    return 0;
}
