#include <Wire.h>
#include <VL6180X.h>
#include <Arduino.h>
#include <TM1637Display.h>

// Module connection pins (Digital Pins)
#define CLK 2
#define DIO 3
#define LOOP_TIME 10    // Loop time in milliseconds
#define FAN_PWM_PIN 6  // PWM pin for the fan
#define SCALING 2

VL6180X sensor;  // Create an instance of the VL6180X class


long duration;  // variable for the duration of sound wave travel
int distance;   // variable for the distance measurement
int prev_distance = 100;

int setpoint = 110;        // Setpoint height in mm
int process_variable = 0;  // Measured height in mm
float error = 0;           // Error between setpoint and process variable
float previous_error = 0;
float integral_error = 0;    // Integral component of the error
float derivative_error = 0;  // Derivative component of the error
double Kp = 0.015;            // Proportional gain 0.03
double Ki = 0.002;           // Integral gain 0.005
double Kd = 0.001;           // Derivative gain 0.001
float PID_output = 0;        // PID output

void setup() {
  // Initialize the serial port for debugging
  Serial.begin(115200);
  pinMode(FAN_PWM_PIN, OUTPUT);
  Wire.begin();
  sensor.init();
  sensor.configureDefault();
  sensor.setScaling(SCALING);
  sensor.setTimeout(500);
}


void loop() {
  
  process_variable = sensor.readRangeSingleMillimeters();

  // Calculate the error
  error = setpoint - process_variable;
  //Serial.println("error= "+String(error));
  // Calculate the integral component of the error
  integral_error += error * LOOP_TIME;

  // Calculate the derivative component of the error
  derivative_error = (error - previous_error) / LOOP_TIME;
  previous_error = error;

  // Calculate the PID output
  PID_output = -Kp * error - Ki * integral_error - Kd * derivative_error;
  //Serial.println("PID_output= "+String(PID_output));
  if (PID_output < 0) {
    PID_output = 0;
  } else if (PID_output > 255) {
    PID_output = 255;
  }
  // Apply the PID output to the process (
  analogWrite(FAN_PWM_PIN, PID_output);
  // Serial.print(millis());
  // Serial.print("setpoint= ");
  Serial.print(setpoint);
  Serial.print(",");
  // Serial.print("sensor= ");
  Serial.print(process_variable);
  Serial.print(",");
  // Serial.print("voltage= ");
  Serial.println(PID_output);
  
  //setpoint=sensorValue/2;
  //Serial.println("Target "+String(setpoint));
  delay(LOOP_TIME);
}