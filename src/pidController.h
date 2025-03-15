// This file contains the PID control logic for the test bench
// Author: Scott Roelker
// March 15, 2025

#include <Arduino.h>
// Debug Flags
#define PID_DEBUG 0


class PIDController{
    public:
        // Implement PID control of motor PWM
        // Based on: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/
        int control(int targetSpeed, int currentSpeed)
        {
          static float error = 0;
          static float prevError = 0;
          static float lastSpeed = 0;

          // current speed can't be 0 or the entire PID loop will break
          if(currentSpeed == 0){
            currentSpeed = 1;
          }

          // Calculate error
          error = targetSpeed - currentSpeed;
          // Calculate integral
          integral += Ki * error;
          // Clamp to usable range
          if(integral> outputMax) integral = outputMax;
          else if(integral < outputMin) integral = outputMin;

          // Calculate derivative
          //derivative = error - prevError; (old way)
          derivative = (currentSpeed-lastSpeed);

          // Calculate output
          //output = Kp * error + Ki * integral + Kd * derivative; (old way)
          output = Kp * error + integral - Kd * derivative; 
          if(output> outputMax) output = outputMax;
          else if(output < outputMin) output = outputMin;

          // Constrain output to PWM range
          output = constrain(output, outputMin, outputMax);

          // Update previous error
          prevError = error;
          lastSpeed = currentSpeed;

          if(PID_DEBUG){
            // Print PID values for debugging
            Serial.print("Target: ");
            Serial.println(targetSpeed);
            Serial.print("Current: ");
            Serial.println(currentSpeed);
            Serial.print("Error: ");
            Serial.println(error);
            Serial.print("Integral: ");
            Serial.println(integral);
            Serial.print("Derivative: ");
            Serial.println(derivative);
           // Serial.print(", Kp: ");
           // Serial.print(Kp);
           // Serial.print(", Ki: ");
           // Serial.print(Ki);
           // Serial.print(", Kd: ");
           // Serial.print(Kd);
           Serial.print("Output: ");
            Serial.println(output);
          }

          return output;
        }
        // Set output limits for PID controller
        void PIDSetOutputLimits(float min, float max)
        {
           if(min > max) return;
           outputMin = min;
           outputMax = max;

           if(output > outputMax) output = outputMax;
           else if(output < outputMin) output = outputMin;

           if(integral> outputMax) integral= outputMax;
           else if(integral< outputMin) integral= outputMin;
        }
        // Set tunings for PID
        void PIDSetTunings(float Kp_in, float Ki_in, float Kd_in)
        {
          if(Kp_in < 0 || Ki_in < 0 || Kd_in < 0) return;
          // Set PID tunings
          Kp = Kp_in;
          Ki = Ki_in * PIDPeriod;
          Kd = Kd_in / PIDPeriod;
        }
        // Constructor
        PIDController(float period, float outMin, float outMax, float Kp_in, float Ki_in, float Kd_in)
        {
          // Set PID period
          PIDPeriod = period;
          // Set output limits
          outputMin = 0;
          outputMax = outMax;
          // Set PID tunings
          Kp = Kp_in;
          Ki = Ki_in * PIDPeriod;
          Kd = Kd_in / PIDPeriod;
          // Initialize PID variables
          integral = 0;
          derivative = 0;
          output = 0;
        }

    private:
        // PID settings
        float PIDPeriod; // Period of PID timer in seconds
        // works for 12 bit resolution at 10Hz 
        //float Kp = 0.05; // Proportional gain
        //float Ki = 0.3*PIDPeriod; // Integral gain per unit time
        //float Kd = 0.3/PIDPeriod; // Derivative gain per unit time
        // works for 16 bit resolution at 10Hz
        //float Kp = 10; // Proportional gain
        //float Ki = 10*PIDPeriod; // Integral gain per unit time
        //float Kd = 3/PIDPeriod; // Derivative gain per unit time
        // Manual Mode (set manually by variable resistors)
        float Kp; // Proportional gain
        float Ki; // Integral gain per unit time
        float Kd; // Derivative gain per unit time

        float integral;
        float derivative;
        float outputMin,outputMax; // Output limits
        int output; // PWM output value
};
