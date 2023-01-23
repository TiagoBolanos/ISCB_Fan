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
const uint8_t SET[] = {
  SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,  // s
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,  // e
  SEG_D | SEG_E | SEG_F | SEG_G,          //t
  SEG_A | SEG_B | SEG_E | SEG_F | SEG_G,  //p
};
const uint8_t ALT[] = {
  SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,  // A
  SEG_D | SEG_E | SEG_F,                          // L
  SEG_D | SEG_E | SEG_F | SEG_G,                  //t
  SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,          //U
};
const uint8_t Set_Kp[] = {
  SEG_A | SEG_B | SEG_E | SEG_F | SEG_G,          //p
  SEG_E | SEG_G,                                  //r
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,  //O
  SEG_A | SEG_B | SEG_E | SEG_F | SEG_G,          //p
};
const uint8_t Set_Ki[] = {
  SEG_C,                                  //i
  SEG_C | SEG_E | SEG_G,                  //n
  SEG_D | SEG_E | SEG_F | SEG_G,          //t
  SEG_A | SEG_C | SEG_D | SEG_E | SEG_F,  //g
};
const uint8_t Set_Kd[] = {
  SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,  // d
  SEG_E | SEG_G,                          //r
  SEG_C | SEG_D | SEG_E,                  //v
  SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,  // d
};
const uint8_t POUT[] = {
  SEG_A | SEG_B | SEG_E | SEG_F | SEG_G,          // P
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,  // O
  SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,          // U
  SEG_D | SEG_E | SEG_F | SEG_G                   // T
};
const uint8_t ERRO[] = {
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,          // E
  SEG_E | SEG_G,                                  //r
  SEG_E | SEG_G,                                  //r
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,  // O
};
TM1637Display display(CLK, DIO);

long duration;  // variable for the duration of sound wave travel
int distance;   // variable for the distance measurement
int prev_distance = 100;
float sensorValue = 0;
int displayed = 0;
float variable = 0;
bool flag = false;

const int buttonPin = 0;           // Button is connected to digital pin 2
int buttonState = 0;               // Variable to store the button state
unsigned long previousMillis = 0;  // Variable to store the previous time
const int debounceDelay = 500;     // Debounce delay in milliseconds
bool wait = false;
unsigned long currentMillis = millis();

int setpoint = 190;        // Setpoint height in mm
int process_variable = 0;  // Measured height in mm
float error = 0;           // Error between setpoint and process variable
float previous_error = 0;
float integral_error = 0;    // Integral component of the error
float derivative_error = 0;  // Derivative component of the error
double Kp = 0.1;            // Proportional gain 0.015
double Ki = 0.002;           // Integral gain 0.002
double Kd = 0.001;           // Derivative gain 0.001
float PID_output = 0; 

void setup() {
  // Initialize the serial port for debugging
  Serial.begin(115200);
  pinMode(FAN_PWM_PIN, OUTPUT);
  pinMode(buttonPin, INPUT);  // Set the button pin as an input
  Wire.begin();
  sensor.init();
  sensor.configureDefault();
  sensor.setScaling(SCALING);
  sensor.setTimeout(500);
  display.setBrightness(0x0f);
}

void show() {
  sensorValue = analogRead(A0);
  setpoint = map(sensorValue, 0, 1023, 30, 375);
  switch (displayed) {
    case 0:  //height
      if (wait) {
        display.setSegments(ALT);  //ALT
      } else {
        display.showNumberDec(process_variable);       
        // Serial.println("Height " + String(process_variable));
      }
      break;
    case 1:  //setpoint
      if (wait) {
        display.setSegments(SET);
      } else {
        display.showNumberDec(setpoint);
        // Serial.println("Target " + String(setpoint));
      }
      break;
    case 2:  //error
      if (wait) {
        display.setSegments(ERRO);
      } else {
        display.showNumberDec(error);
        // Serial.println("Error= " + String(error));
      }
      break;
    case 3:  //PID_output
      if (wait) {
        display.setSegments(POUT);
      } else {
        display.showNumberDec(PID_output);
        // Serial.println("PID_output= " + String(PID_output));
      }
      break;
    case 4:  //porportional K
      if (wait) {
        display.setSegments(Set_Kp);
      } else {
        display.showNumberDec(Kp * 1000);
        // Serial.println("Kp= " + String(Kp));
      }
      break;
    case 5:  //integral K
      if (wait) {
        display.setSegments(Set_Ki);
      } else {
        display.showNumberDec(Ki * 1000);      
        // Serial.println("Ki= " + String(Ki));
      }
      break;
    case 6:  //derivative K
      if (wait) {
        display.setSegments(Set_Kd);
      } else {
        display.showNumberDec(Kd * 1000);      
        // Serial.println("Kp= " + String(Kd));
      }
      break;
  }
}
void loop() {
  buttonState = digitalRead(buttonPin);
  sensorValue = analogRead(A0);
  currentMillis = millis();
  // Check if the button has been pressed
  if (buttonState == LOW) {
    // Get the current time
    previousMillis = currentMillis;
    if (!wait) { displayed = (displayed + 1) % 3; }
    // change to 7 to display Ks
  }
  // Check if the debounce delay has passed
  if (currentMillis - previousMillis > debounceDelay) {
    wait = false;
  } else {
    wait = true;
  }
  process_variable = sensor.readRangeSingleMillimeters();
  show();

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
  Serial.print(millis());
  Serial.print(",");
  Serial.print(process_variable);
  Serial.print(",");
  Serial.print(setpoint);
  Serial.print(",");
  Serial.println((PID_output/255)*100);
  //Serial.println("Target "+String(setpoint));
  delay(LOOP_TIME);
}