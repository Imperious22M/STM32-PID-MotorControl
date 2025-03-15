// Create a simple motor controller for the encoder-driven motor we are using

#include <Arduino.h>

class motorController{
    public:
        // Maps enum entried to IN1/2 states
        enum motorDirection{
          FORWARD,
          BACKWARD,
          OFF
        };

        motorController(int ena, int in1, int in2, int pwmRes): ENA(ena), IN1(in1), IN2(in2)
        {
          // Set PWM resolution
          analogWriteResolution(pwmRes); // Set PWM resolution

          // Set motor control pins
          pinMode(ENA, OUTPUT);
          pinMode(IN1, OUTPUT);
          pinMode(IN2, OUTPUT);
          // Set motor to off
          digitalWrite(IN1, motorDir[OFF][0]);
          digitalWrite(IN2, motorDir[OFF][1]);
        }

        void setDirection(motorDirection dir)
        {
          // Set motor direction
          digitalWrite(IN1, motorDir[dir][0]);
          digitalWrite(IN2, motorDir[dir][1]);
          direction = dir;
        }

        motorDirection getDirection()
        {
          return direction;
        }

        void setPWMOut(int pwm)
        {
          analogWrite(ENA, pwm);
        }

    private:
        // H Bridge pins to control motor
        int ENA; // Enable/Disable H-bridge
        int IN1; // IN1/2 are direction
        int IN2; 
        motorDirection direction;
        const int motorDir[3][2]{
            {1,0},
            {0,1},
            {0,0}
        };

};