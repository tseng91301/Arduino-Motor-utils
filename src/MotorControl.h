#pragma once

#include<Arduino.h>


class Motor {
    private:
        // Pin Definition
        bool have_encoder = false;
        bool have_encoder_pin = false;
        int in1;
        int in2;
        int encoderA;
        int encoderB;
        
        // Switch
        bool motor_enabled = true;

        // Encoder Reading
        unsigned long read_speed_interval = 150;
        
        double encoderSpeed = 0;

        // PID Control constants
        double kP = 1.2;
        double kI = 0.004;
        double kD = 0.0;
        double e_prev = 0;
        double e = 0;
        double e_integral = 0;

        // Speed Control
        int target_speed = 0;
        void _turn(int spd);
        void update_speed();
        double motor_speed = 0;

        // init
        void init();

        // Serial command callback
        bool have_serial_callback = false;
        uint8_t callback_byte;

    public:
        bool reversed = false;

        volatile long encoderPos = 0;
        // Construction
        Motor(int in1, int in2); // Without encoder
        Motor(int in1, int in2, int encoderA, int encoderB); // With encoder
        Motor(int in1, int in2, int encoderA, int encoderB, void (*isr)()); // With encoder and ISR

        // Public Function Call
        void attach_interrupt_isr(void (*isr)());
        void set_callback_byte(uint8_t callback_byte) { this->callback_byte = callback_byte; this->have_serial_callback = true; }

        void set_speed(int inp, bool callback=true);
        double get_current_speed();

        // encoder function
        void encoder();

        // Loop Service
        void service();

};

class DualMotor {
    private:
        Motor *motorL;
        Motor *motorR;

        double L_calibration = 1.0; // |Max| 1
        double R_calibration = 1.0; // |Max| 1

        void set_speed(int spdL, int spdR) {
            // Serial.println(spdL);
            // Serial.println(spdR);
            motorL->set_speed(spdL);
            motorR->set_speed(spdR);
        }
    public:
        
        // Settings params
        double speed = 0.0; // |Max| 1
        double L_weight = 0.0; // |Max| 1
        double R_weight = 0.0; // |Max| 1

        DualMotor(Motor *motorL, Motor *motorR): motorL(motorL), motorR(motorR) {}

        void set_calibration(double L_calibration, double R_calibration) {
            // Calibrate Left and Right speed.
            // input any floating number bigger than 0.0
            // To ensure the speed is the max output, we will set the big one to 1.0, and the other will be divided, respectively.
            if (L_calibration <= 0.0 || R_calibration <= 0.0) {
                return;
            }

            if (L_calibration > R_calibration) {
                R_calibration = R_calibration / L_calibration;
                L_calibration = 1.0;
            } else {
                L_calibration = L_calibration / R_calibration;
                R_calibration = 1.0;
            }
        }

        void set_direction(int direction) {
            // Input a direction angle between [-180, 180]
            // -180: L_weight = 1.0, R_weight = -1.0
            // 0: L_weight = 0.0, R_weight = 0.0
            // 180: L_weight = -1.0, R_weight = 1.0
            if (direction < -180 || direction > 180) {
                return;
            }

            if (direction <= 0 && direction >= -180) {
                L_weight = 1.0;
                R_weight = cos(direction * PI / 180.0);
            } else if (direction > 0 && direction <= 180) {
                L_weight = cos(direction * PI / 180.0);
                R_weight = 1.0;
            }

            set_speed(255 * speed * L_weight * L_calibration, 255 * speed * R_weight * R_calibration);
            return;
        }
        
        void service() {
            motorL->service();
            motorR->service();
        }

};