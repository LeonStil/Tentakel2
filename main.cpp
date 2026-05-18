#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "hardware/pwm.h"

// PID
const float KP = 5.0;
const float KI = 0.01;
const float KD = 0.5;
const float SETPOINT = 0.0;

float previousError = 0.0;
float integral = 0.0;

// Welke pins zijn dit???
const int MOTOR_PIN_P_1 = 21;
const int MOTOR_PIN_P_2 = 22;
const int MOTOR_PIN_Q_1 = 27;
const int MOTOR_PIN_Q_2 = 26;

const float MAX_PID_OUTPUT = 140.0;
const float MAX_PWM = 80.0; // battery runs on 8V so 80% makes maximum output 6.4 V

const float MOTOR_START_PWM = 30.0;

// LED
#define LED_GREEN 13
#define LED_RED 12
// I2C defines
#define I2C_PORT i2c0
#define MPU6050_ADDR 0x68
// SERVO
#define SERVO_PIN 14
//For PWM
#define MAXIMUM_LEVEL 1000 // used to be 20000

// MPU6050 register addresses
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_SMPLRT_DIV 0x19
#define WHO_AM_I_REG 0x75

// Sensitivity scale factors for different ranges
#define ACCEL_SCALE_FACTOR_2G 16384.0  // for ±2g
#define ACCEL_SCALE_FACTOR_4G 8192.0   // for ±4g
#define ACCEL_SCALE_FACTOR_8G 4096.0   // for ±8g
#define ACCEL_SCALE_FACTOR_16G 2048.0  // for ±16g

#define GYRO_SCALE_FACTOR_250DPS 131.0    // for ±250 degrees per second
#define GYRO_SCALE_FACTOR_500DPS 65.5     // for ±500 degrees per second
#define GYRO_SCALE_FACTOR_1000DPS 32.8    // for ±1000 degrees per second
#define GYRO_SCALE_FACTOR_2000DPS 16.4    // for ±2000 degrees per second

// Select the desired scale factor
#define ACCEL_SCALE_FACTOR ACCEL_SCALE_FACTOR_4G  // Change this to the desired accelerometer range
#define GYRO_SCALE_FACTOR GYRO_SCALE_FACTOR_250DPS // Change this to the desired gyroscope range

// Corresponding configuration values
#define ACCEL_CONFIG_VALUE 0x08  // for ±4g
#define GYRO_CONFIG_VALUE 0x00  // for ±250 degrees per second
#define SAMPLE_RATE_DIV 0  // Sample rate = 1kHz / (1 + 1) = 500Hz

//Prototypes of functions
float constrain(float value, float minVal, float maxVal);
void setupMotorPWM(int pin);
float updatePid(float setpoint, float measuredValue, float dt, float &pValue, float &iValue, float &dValue);
void convertPidToMotor(float pidOutput, float &pwmA, float &pwmB, float &motorOutput);
float addMotorStartPower(float pwm);
void driveMotors(float pwmA, float pwmB);
void mpu6050_reset();
void mpu6050_configure();
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);
void debuggingLEDs(float angle);

int main() {
    // Initialize chosen serial port
    stdio_init_all();

    while (!stdio_usb_connected()) {
    sleep_ms(100);
    }
    printf("Starting... \n");
    // Init LED
    gpio_init(LED_GREEN);
    gpio_set_dir(LED_GREEN, GPIO_OUT);
    gpio_init(LED_RED);
    gpio_set_dir(LED_RED, GPIO_OUT);

    sleep_ms(500);

    // Init MOTORS
    setupMotorPWM(MOTOR_PIN_P_1);
    setupMotorPWM(MOTOR_PIN_P_2);
    setupMotorPWM(MOTOR_PIN_Q_1);
    setupMotorPWM(MOTOR_PIN_Q_2);
    printf("Succesfully setup motors \n");

    sleep_ms(500);

    // Initialize I2C
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);
    printf("initialized i2c port \n");

    // Reset and configure MPU6050
    mpu6050_reset(); // Is de interupt pin verbonden en gedefinieerd in code
    mpu6050_configure();
    printf("Configured MPU6050 \n");

    uint8_t who_am_i = 0;
    uint8_t who_reg = WHO_AM_I_REG;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &who_reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, &who_am_i, 1, false);
    printf("MPU6050 WHO_AM_I: 0x%02X\n", who_am_i);

    if (who_am_i != 0x68) {
        while (1) {
            printf("MPU6050 not found!\n");
        };
    }

    int16_t accel[3], gyro[3], temp;
    float angle_data_buffer[5];
    float roll = 0.0f;
    float pitch = 0.0f;
    absolute_time_t last_time = get_absolute_time();
    

    // Servo PWM setup
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);

    pwm_config config = pwm_get_default_config();

    // 125 MHz / 125 = 1 MHz
    pwm_config_set_clkdiv(&config, 125.0f);

    // 1 MHz / 20000 = 50 Hz
    pwm_config_set_wrap(&config, MAXIMUM_LEVEL);

    pwm_init(slice_num, &config, true);
    
    while (1) {
        mpu6050_read_raw(accel, gyro, &temp);

        // Convert raw accelerometer values to g
        float accel_g[3];
        accel_g[0] = accel[0] / ACCEL_SCALE_FACTOR;
        accel_g[1] = accel[1] / ACCEL_SCALE_FACTOR;
        accel_g[2] = accel[2] / ACCEL_SCALE_FACTOR;

        float ax = accel_g[0];
        float ay = accel_g[1];
        float az = accel_g[2];

        // Convert raw gyroscope values to degrees per second
        float gyro_dps[3];
        gyro_dps[0] = gyro[0] / GYRO_SCALE_FACTOR;
        gyro_dps[1] = gyro[1] / GYRO_SCALE_FACTOR;
        gyro_dps[2] = gyro[2] / GYRO_SCALE_FACTOR;

        float gx = gyro_dps[0];
        float gy = gyro_dps[1];
        float gz = gyro_dps[2];

        absolute_time_t current_time = get_absolute_time();
        double dt = absolute_time_diff_us(last_time, current_time) / 1000000.0;
        last_time = current_time;

        float roll_acc = atan2f(ay, az) * 180.0f / M_PI;

        float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;

        // Gyroscope integration
        roll += gx * dt;
        pitch += gy * dt;

        // Complementary filter
        roll = 0.98f * roll + 0.02f * roll_acc;
        pitch = 0.98f * pitch + 0.02f * pitch_acc;
        //printf("roll %.2f | pitch %.2f \n", roll, pitch);

    
        // CONSTANT OFFSET MAKE SURE THIS IS CORRECT
        float roll_out = roll + 2.3;
        float pitch_out = pitch + 4.4;
        
        float pValue;
        float iValue;
        float dValue;

        float pidOutput = updatePid(
        SETPOINT,
        roll_out,
        dt,
        pValue,
        iValue,
        dValue
        );

        float pwmA;
        float pwmB;
        float motorOutput;

        convertPidToMotor(
        pidOutput,
        pwmA,
        pwmB,
        motorOutput
        );

        float adjustedPwmA = addMotorStartPower(pwmA);
        float adjustedPwmB = addMotorStartPower(pwmB);

        driveMotors(adjustedPwmA, adjustedPwmB);

        //pwm_set_gpio_level(SERVO_PIN, 100);
        
        //printf( "aX = %.2f g | aY = %.2f g | aZ = %.2f g | gX = %.2f dps | gY = %.2f dps | gZ = %.2f dps | temp = %.2f°C\n", accel_g[0], accel_g[1], accel_g[2], gyro_dps[0], gyro_dps[1], gyro_dps[2], temp / 340.00 + 36.53 );
        // Print orientation
        //printf("%.2f,%.2f\n", roll_out, pitch_out);

        printf( "roll = %.2f deg | pitch = %.2f deg | pwmA = %.2f | pwmB = %.2f | dt = %.5f ax %.5f \n", roll_out, pitch_out, adjustedPwmA, adjustedPwmB, dt, ax);

        //sleep_ms(1);
    }
    return 0;
}

void debuggingLEDs(float angle) {
    if (angle > -3 && angle < 3) {
        gpio_put(LED_GREEN, 1);
        gpio_put(LED_RED, 0);
    }
    else {
        gpio_put(LED_GREEN, 0);
        gpio_put(LED_RED, 1);
    }
}

float constrain(float value, float minVal, float maxVal) {
    // This function works in the same way as the constrain function on arduino
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

void setupMotorPWM(int pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(pin);

    pwm_config config = pwm_get_default_config();

    // PWM clock divider
    pwm_config_set_clkdiv(&config, 125.0f);

    // PWM wrap value
    pwm_config_set_wrap(&config, MAXIMUM_LEVEL);

    pwm_init(slice_num, &config, true);

    pwm_set_gpio_level(pin, 0);
}

float updatePid(float setpoint, float measuredValue, float dt, float &pValue, float &iValue, float &dValue) {
  float error = setpoint - measuredValue;

  integral += error * dt;

  float derivative = 0.0;
  if (dt > 0.0) {
    derivative = (error - previousError) / dt;
  }

  pValue = KP * error;
  iValue = KI * integral;
  dValue = KD * derivative;

  previousError = error;

  return pValue + iValue + dValue;
}

void convertPidToMotor(float pidOutput, float &pwmA, float &pwmB, float &motorOutput) {
  motorOutput = pidOutput / MAX_PID_OUTPUT * MAX_PWM;

  motorOutput = constrain(motorOutput, -MAX_PWM, MAX_PWM);

  if (motorOutput > 0.0) {
    pwmA = motorOutput;
    pwmB = 0.0;
  } else if (motorOutput < 0.0) {
    pwmA = 0.0;
    pwmB = abs(motorOutput);
  } else {
    pwmA = 0.0;
    pwmB = 0.0;
  }
}

float addMotorStartPower(float pwm) {
  if (pwm <= 0.0) {
    return 0.0;
  }

  float adjustedPwm = MOTOR_START_PWM + (pwm / 100.0) * (MAX_PWM - MOTOR_START_PWM);

  return constrain(adjustedPwm, 0.0, MAX_PWM);
}

void driveMotors(float pwmA, float pwmB) {
    float adjustedPwmA = addMotorStartPower(pwmA);
    float adjustedPwmB = addMotorStartPower(pwmB);

    // Convert 0-100% PWM to 0-MAXIMUM_LEVEL
    uint16_t pwmAValue = (uint16_t)((adjustedPwmA / 100.0f) * MAXIMUM_LEVEL);
    uint16_t pwmBValue = (uint16_t)((adjustedPwmB / 100.0f) * MAXIMUM_LEVEL);

    pwmAValue = (uint16_t)constrain(pwmAValue, 0, MAXIMUM_LEVEL);
    pwmBValue = (uint16_t)constrain(pwmBValue, 0, MAXIMUM_LEVEL);

    // Forward direction
    if (adjustedPwmA > 0.0f) {

        pwm_set_gpio_level(MOTOR_PIN_P_1, pwmAValue);
        pwm_set_gpio_level(MOTOR_PIN_Q_1, pwmAValue);

        pwm_set_gpio_level(MOTOR_PIN_P_2, 0);
        pwm_set_gpio_level(MOTOR_PIN_Q_2, 0);
    }

    // Reverse direction
    else if (adjustedPwmB > 0.0f) {

        pwm_set_gpio_level(MOTOR_PIN_P_1, 0);
        pwm_set_gpio_level(MOTOR_PIN_Q_1, 0);

        pwm_set_gpio_level(MOTOR_PIN_P_2, pwmBValue);
        pwm_set_gpio_level(MOTOR_PIN_Q_2, pwmBValue);
    }

    // Stop
    else {

        pwm_set_gpio_level(MOTOR_PIN_P_1, 0);
        pwm_set_gpio_level(MOTOR_PIN_Q_1, 0);

        pwm_set_gpio_level(MOTOR_PIN_P_2, 0);
        pwm_set_gpio_level(MOTOR_PIN_Q_2, 0);
    }
}

void mpu6050_reset() {
    uint8_t reset[] = {REG_PWR_MGMT_1, 0x80};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, reset, 2, false);
    sleep_ms(200);
    uint8_t wake[] = {REG_PWR_MGMT_1, 0x00};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, wake, 2, false);
    sleep_ms(200);
}

void mpu6050_configure() {
    // Set accelerometer range
    uint8_t accel_config[] = {REG_ACCEL_CONFIG, ACCEL_CONFIG_VALUE};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, accel_config, 2, false);

    // Set gyroscope range
    uint8_t gyro_config[] = {REG_GYRO_CONFIG, GYRO_CONFIG_VALUE};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, gyro_config, 2, false);

    // Set sample rate
    uint8_t sample_rate[] = {REG_SMPLRT_DIV, SAMPLE_RATE_DIV};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, sample_rate, 2, false);
}

void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[14];
    uint8_t reg = REG_ACCEL_XOUT_H;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 14, false);

    accel[0] = (buffer[0] << 8) | buffer[1];
    accel[1] = (buffer[2] << 8) | buffer[3];
    accel[2] = (buffer[4] << 8) | buffer[5];
    *temp = (buffer[6] << 8) | buffer[7];
    gyro[0] = (buffer[8] << 8) | buffer[9];
    gyro[1] = (buffer[10] << 8) | buffer[11];
    gyro[2] = (buffer[12] << 8) | buffer[13];
}