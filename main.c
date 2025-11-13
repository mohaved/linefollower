#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/mutex.h"
#include "line_follower.h"

extern volatile bool calibration_requested;
extern volatile bool start_stop_requested;
extern volatile bool core1_ready;

static inline int16_t constrain(int16_t val, int16_t min, int16_t max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

SharedControl shared_control = { .mutex = {}, .pid_output = 0.0f, .new_data = false };

void core0_motor_control() {
    absolute_time_t next_update = get_absolute_time();

    while (1) {
        mutex_enter_blocking(&shared_control.mutex);
        bool has_new_data = shared_control.new_data;
        float pid_output = shared_control.pid_output;
        uint16_t base_speed = shared_control.base_speed;
        shared_control.new_data = false;
        mutex_exit(&shared_control.mutex);

        if (has_new_data && get_robot_state() == RUNNING) {
            float correction_scale = (float)base_speed / PWM_MAX; 
            int16_t correction = (int16_t)(pid_output * PID_SCALE * correction_scale);

            int16_t left_speed, right_speed;
            if (correction > 0) {
                left_speed = base_speed;
                right_speed = base_speed - (correction * TURN_ASYMMETRY);
            } else {
                left_speed = base_speed + (correction * TURN_ASYMMETRY); 
                right_speed = base_speed;
            }
            int16_t min_speed = base_speed > HIGH_SPEED_THRESH ? MIN_SPEED_HIGH : MIN_SPEED_LOW;

            left_speed = constrain(left_speed, min_speed, PWM_MAX);
            right_speed = constrain(right_speed, min_speed, PWM_MAX);

            static int16_t prev_left_speed = 0;
            static int16_t prev_right_speed = 0;
            const int16_t max_accel = 20; 

            left_speed = constrain(left_speed,
                prev_left_speed - max_accel,
                prev_left_speed + max_accel);
            right_speed = constrain(right_speed,
                prev_right_speed - max_accel,
                prev_right_speed + max_accel);

            prev_left_speed = left_speed;
            prev_right_speed = right_speed;

            mutex_enter_blocking(&motor_mutex);
            set_motor_speeds(left_speed, right_speed);
            mutex_exit(&motor_mutex);
        }

        next_update = delayed_by_us(next_update, 500);
        sleep_until(next_update);
    }
}

int main() {
    stdio_init_all();
    mutex_init(&shared_control.mutex);
    init_line_follower();

    printf("DEBUG: Line follower robot initialized\n");
    printf("DEBUG: Press CALIBRATION button to calibrate sensors\n");
    printf("DEBUG: Press START/STOP button to start/stop the robot\n");

    multicore_launch_core1(core1_main);

    while (!core1_ready) {
        sleep_ms(10);
    }
    printf("DEBUG: Core 1 ready, starting main control loop\n");

    while (1) {
        if (calibration_requested) {
            calibration_requested = false;
            printf("DEBUG: Calibration button pressed - starting sensor calibration\n");
            set_robot_state(CALIBRATING);
            printf("DEBUG: Calibration complete\n");
        }

        if (start_stop_requested) {
            start_stop_requested = false; 
            RobotState current_state = get_robot_state();
            if (current_state == RUNNING) {
                printf("DEBUG: Stop button pressed - stopping robot\n");
                set_robot_state(STOPPED);
            } else if (current_state == STOPPED) {
                printf("DEBUG: Start button pressed - starting robot\n");
                set_robot_state(RUNNING);
            }
        }
        sleep_ms(10);
    }
    return 0;
}
