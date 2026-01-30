#pragma once
#include <Arduino.h>

/* =========================
   PWM pins
   ========================= */
constexpr uint8_t PWM_MOTOR_LEFT  = 5;
constexpr uint8_t PWM_MOTOR_RIGHT = 6;

/* =========================
   Direction / digital pins
   ========================= */
constexpr uint8_t DIR_MOTOR_LEFT  = 7;
constexpr uint8_t DIR_MOTOR_RIGHT = 8;

/* =========================
   Sensors / inputs
   ========================= */
constexpr uint8_t ENCODER_LEFT_A  = 2;  // interrupt
constexpr uint8_t ENCODER_RIGHT_A = 3;  // interrupt

/* =========================
   LEDs / misc
   ========================= */
constexpr uint8_t LED_STATUS = LED_BUILTIN;