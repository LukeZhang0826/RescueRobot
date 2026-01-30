#ifndef MTE380_CONFIG_H
#define MTE380_CONFIG_H

#include <Arduino.h>

/*
    config.h - pin & communication interface mapping template
    Usage:
        - Override any CONFIG_* macro in your build flags or before including this header.
        - Use the constexpr values below in your code for type safety.
*/

// --------- Pin defaults----------
#ifndef CONFIG_PIN_SERVO_A
#define CONFIG_PIN_SERVO_A 46
#endif

#ifndef CONFIG_PIN_SERVO_B
#define CONFIG_PIN_SERVO_B 45   
#endif

#ifndef CONFIG_PIN_SERVO_C
#define CONFIG_PIN_SERVO_C 44
#endif

#ifndef CONFIG_PIN_MOTOR_A_PWM
#define CONFIG_PIN_MOTOR_A_PWM 13
#endif

#ifndef CONFIG_PIN_MOTOR_A_DIR_1
#define CONFIG_PIN_MOTOR_A_DIR_1 12
#endif

#ifndef CONFIG_PIN_MOTOR_A_DIR_2
#define CONFIG_PIN_MOTOR_A_DIR_2 11
#endif

#ifndef CONFIG_PIN_MOTOR_B_PWM
#define CONFIG_PIN_MOTOR_B_PWM 7
#endif

#ifndef CONFIG_PIN_MOTOR_B_DIR_1
#define CONFIG_PIN_MOTOR_B_DIR_1 9
#endif

#ifndef CONFIG_PIN_MOTOR_B_DIR_2
#define CONFIG_PIN_MOTOR_B_DIR_2 8
#endif

#ifndef CONFIG_PIN_MOTOR_DRIVER_STBY
#define CONFIG_PIN_MOTOR_DRIVER_STBY 10
#endif

#ifndef CONFIG_PIN_ENCODER_A_1
#define CONFIG_PIN_ENCODER_A_1 52
#endif

#ifndef CONFIG_PIN_ENCODER_A_2
#define CONFIG_PIN_ENCODER_A_2 50
#endif

#ifndef CONFIG_PIN_ENCODER_B_1
#define CONFIG_PIN_ENCODER_B_1 5
#endif

#ifndef CONFIG_PIN_ENCODER_B_2
#define CONFIG_PIN_ENCODER_B_2 6
#endif

// --------- constexpr mappings  ----------
constexpr uint8_t PIN_SERVO_A        = CONFIG_PIN_SERVO_A;
constexpr uint8_t PIN_SERVO_B        = CONFIG_PIN_SERVO_B;
constexpr uint8_t PIN_SERVO_C        = CONFIG_PIN_SERVO_C;

constexpr uint8_t PIN_ENCODER_A_1   = CONFIG_PIN_ENCODER_A_1;
constexpr uint8_t PIN_ENCODER_A_2   = CONFIG_PIN_ENCODER_A_2;
constexpr uint8_t PIN_ENCODER_B_1   = CONFIG_PIN_ENCODER_B_1;
constexpr uint8_t PIN_ENCODER_B_2   = CONFIG_PIN_ENCODER_B_2;

constexpr uint8_t PIN_MOTOR_A_PWM   = CONFIG_PIN_MOTOR_A_PWM;
constexpr uint8_t PIN_MOTOR_A_DIR_1  = CONFIG_PIN_MOTOR_A_DIR_1;
constexpr uint8_t PIN_MOTOR_A_DIR_2  = CONFIG_PIN_MOTOR_A_DIR_2;
constexpr uint8_t PIN_MOTOR_B_PWM   = CONFIG_PIN_MOTOR_B_PWM;
constexpr uint8_t PIN_MOTOR_B_DIR_1  = CONFIG_PIN_MOTOR_B_DIR_1;
constexpr uint8_t PIN_MOTOR_B_DIR_2  = CONFIG_PIN_MOTOR_B_DIR_2;
constexpr uint8_t PIN_MOTOR_DRIVER_STBY  = CONFIG_PIN_MOTOR_DRIVER_STBY;

// --------- Simple helper functions ----------
static inline void configPinsAsDefault() {
    pinMode(PIN_SERVO_A, OUTPUT);
    pinMode(PIN_SERVO_B, OUTPUT);
    pinMode(PIN_SERVO_C, OUTPUT);
    pinMode(PIN_ENCODER_A_1, INPUT);
    pinMode(PIN_ENCODER_A_2, INPUT);
    pinMode(PIN_ENCODER_B_1, INPUT);
    pinMode(PIN_ENCODER_B_2, INPUT);
    pinMode(PIN_MOTOR_A_PWM, OUTPUT);
    pinMode(PIN_MOTOR_A_DIR_1, OUTPUT);
    pinMode(PIN_MOTOR_A_DIR_2, OUTPUT);
    pinMode(PIN_MOTOR_B_PWM, OUTPUT);
    pinMode(PIN_MOTOR_B_DIR_1, OUTPUT);
    pinMode(PIN_MOTOR_B_DIR_2, OUTPUT);
    pinMode(PIN_MOTOR_DRIVER_STBY, OUTPUT);
}

#endif // MTE380_CONFIG_H