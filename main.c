#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/mutex.h"
#include "robot/line_follower.h"

// Shared data structure between cores
// Define shared_control instance declared in header
SharedControl shared_control = { .mutex = {}, .pid_output = 0.0f, .new_data = false };

// Core 0: Main control loop - Handles motor control
void core0_motor_control() {
    const int16_t base_speed = 200;
    
    while (1) {
        mutex_enter_blocking(&shared_control.mutex);
        bool has_new_data = shared_control.new_data;
        float pid_output = shared_control.pid_output;
        shared_control.new_data = false;
        mutex_exit(&shared_control.mutex);

        if (has_new_data && get_robot_state() == RUNNING) {
            // Calculate motor speeds based on PID output
            // Scale PID to PWM units (assume pid_output ~ few units); use scale factor
            int16_t correction = (int16_t)(pid_output * 100.0f); // tuning factor
            int16_t left_speed = base_speed - correction;
            int16_t right_speed = base_speed + correction;
            
            // Clamp and apply motor speeds inside motor mutex
            mutex_enter_blocking(&motor_mutex);
            set_motor_speeds(left_speed, right_speed);
            mutex_exit(&motor_mutex);
        }
        
        // Small delay to prevent overwhelming the motors
        sleep_us(100);
    }
}

int main() {
    // Initialize stdio for debug output
    stdio_init_all();
    
    // Initialize mutex
    mutex_init(&shared_control.mutex);
    
    // Initialize the line follower system
    init_line_follower();
    
    printf("Line follower robot initialized\n");
    printf("Press CALIBRATION button to calibrate sensors\n");
    printf("Press START/STOP button to start/stop the robot\n");
    
    // Start core1 for sensor processing
    multicore_launch_core1(core1_main);
    
    // Run motor control on core0
    core0_motor_control();
    
    return 0;
}