#include <Arduino.h>
#include <MotorControl.h>

#define MOTOR_1 5
#define MOTOR_2 6
#define ENCODER_A 2
#define ENCODER_B 4

Motor *motor = nullptr;

/// @brief Simple ISR function entry
void encoderISR() {
    if (motor) motor->encoder();
}

void set_motor_speed(int spd) {
  motor->set_speed(spd);
}

void setup() {
  motor = new Motor(MOTOR_1, MOTOR_2, ENCODER_A, ENCODER_B, encoderISR);
  // put your setup code here, to run once:
}

void loop() {
  motor->service();
  // put your main code here, to run repeatedly:
}