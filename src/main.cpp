#include <Arduino.h>
#include <stm32yyxx_ll_tim.h>
#include <quadratureDecoder.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MCP2515.h>


/*
  Quadrature Decoder example using hardware timer 1 (Ch1 and Ch2)
  4x mode, with hardware filtering and edge detection
*/

#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x01090000)
#error "Due to API change, this sketch is compatible with STM32_CORE_VERSION  >= 0x01090000"
#endif


// Debug Flags
#define PID_DEBUG 0


// Qudrature decoder pin definitions
#define T1CH1_Pin PA8 // Quadrature A channel input pin
#define T1CH2_Pin PA9 // Quadrature B channel input pin

// Motor pin definitions
#define ENA PB8
#define IN1 PB7
#define IN2 PB6
#define ticksPerRevolution 1800 // Number of "ticks" per revolution

// Speed update rate in HZ
#define SPEED_UPDATE_RATE_HZ 20
const float speedTimeRate =  1.0/SPEED_UPDATE_RATE_HZ; // Period of update rate
volatile uint16_t prevTickCnt = 0; // Previously stored Tick Count, initialized to zero upon reset
volatile float ticksPerSec = 0; // Revolutions per second of the encoder shaft

// PID settings
int targetTick = 1800;
// works for 12 bit resolution at 10Hz 
//float Kp = 0.05; // Proportional gain
//float Ki = 0.3*speedTimeRate; // Integral gain per unit time
//float Kd = 0.3/speedTimeRate; // Derivative gain per unit time
// works for 16 bit resolution at 10Hz
//float Kp = 10; // Proportional gain
//float Ki = 10*speedTimeRate; // Integral gain per unit time
//float Kd = 3/speedTimeRate; // Derivative gain per unit time
// Manual Mode (set manually by variable resistors)
float Kp = 0 ; // Proportional gain
float Ki = 0*speedTimeRate; // Integral gain per unit time
float Kd = 0/speedTimeRate; // Derivative gain per unit time

float integral = 0;
float derivative = 0;
float outputMin,outputMax; // Output limits
int output = 0; // PWM output value

// PWM settings
#define PWM_RES 16 // PWM resolution in bits
const int PWM_MAX = (1 << PWM_RES) - 1; // Maximum PWM value

// Variable Resistor Pins
#define TickRes PB1
#define KpRes PB0
#define KiRes PA7
#define KdRes PA6

void calcTickPerSec();
void sendCANMessage();
void PIDControl(int targetSpeed, int currentSpeed);
void PIDSetOutputLimits(float min, float max);
void PIDSetTunings(float Kp, float Ki, float Kd);

// Create an QuadratureDecoder object, this wraps around the functionality of the decoder
// The Timer used MUST support Quadrature decoding, otherwise the functions will not work
QuadratureDecoder quadDecoder(TIM1, T1CH1_Pin, T1CH2_Pin);
// Speed timer used to calculate speed and run PID loop
HardwareTimer *SpeedTimer = new HardwareTimer(TIMER_SERVO);

// Calculates ticks/second at  a rate of SPEED_UPDATE_HZ
// Use direction as discriminant and account for over/underflow
void calcTickPerSecSimple()
{
  uint32_t curTickCnt = quadDecoder.getCountQuad(TICK_FORMAT);
  //bool direction = quadDecoder.getDirBit(); // 0 = forward, 1 = backward 
  static int dirAvg = 0;
  bool effectiveDirection = 0; // same as direction, but averaged over some refresh cycles
  const int averageCycles = 5; // Number of cycles to average over
  // Average direction over some refresh cycles
  // HARDCODED because of erroneous behavior of the encoder
  bool direction = 0;  
  effectiveDirection = 0; // HARDCODED because of erroneous behavior of the encoder

  if(prevTickCnt == curTickCnt){
      ticksPerSec = 0;
  }else {
    // Calculate Ticks/Second based on direction
    if(effectiveDirection == 0){

      // Account for overflow
      if(prevTickCnt>curTickCnt){
        ticksPerSec = (curTickCnt + (UINT16_MAX - prevTickCnt - 1)) / (speedTimeRate);
      }else{
        ticksPerSec = (curTickCnt - prevTickCnt) / (speedTimeRate);
      }

    } else { 

      // TODO OVERFLOW ACCOUNTING for reverse direction
      //digitalWrite(PB5,1);
      //ticksPerSec = (prevTickCnt - curTickCnt) / (speedTimeRate);
    }


    if(ticksPerSec>UINT16_MAX){
      Serial.println("Overflow error! Ticks per second is too high!");
      Serial.print("Ticks per second: ");
      Serial.println(ticksPerSec);  
      Serial.print("Current tick count: ");
      Serial.println(curTickCnt);
      Serial.print("Previous tick count: ");
      Serial.println(prevTickCnt);
      Serial.print("Direction: ");
      Serial.println(direction);
      Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
      Serial.println();
    }
    
    prevTickCnt = curTickCnt;

  }

  //Serial.print("millis:");
  //Serial.println(millis());
  // run PID loop
  PIDControl(targetTick, ticksPerSec); // Set target speed to 2000 RPM
}
// Debug print of Encoder readings
void debugEncoderReadings(){
  //Serial.print("Encoder Count:");
  //Serial.println(quadDecoder.getCountQuad(TICK_FORMAT));
  Serial.print("Direction:");
  //Serial.println(quadDecoder.getDirBit()? "Backward":"Forward");
  Serial.println(quadDecoder.getDirBit());
  Serial.print("Ticks_per_second:");
  Serial.println(ticksPerSec);
  Serial.print("RPS:");
  Serial.println(ticksPerSec/ticksPerRevolution);
  Serial.print("RPM:");
  Serial.println(ticksPerSec/ticksPerRevolution*60);
  //Serial.println("~~~~");
}
// Implement PID control of motor PWM
// Based on: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/
void PIDControl(int targetSpeed, int currentSpeed)
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
  //derivative = error - prevError;
  derivative = (currentSpeed-lastSpeed);
  
  // Calculate output
  //output = Kp * error + Ki * integral + Kd * derivative; (old way)
  output = Kp * error + integral - Kd * derivative; 
  if(output> outputMax) output = outputMax;
  else if(output < outputMin) output = outputMin;

  // Constrain output to PWM range
  output = constrain(output, 0, PWM_MAX);
  // Set motor speed
  analogWrite(ENA, output);
  //Serial.println(output);

  // Update previous error
  prevError = error;
  lastSpeed = currentSpeed;

  if(PID_DEBUG){
    // Print PID values for debugging
    Serial.print("Target Speed: ");
    Serial.print(targetSpeed);
    Serial.print(" RPM, Current Speed: ");
    Serial.print(currentSpeed);
    Serial.print(" RPM, Error: ");
    Serial.print(error);
    Serial.print(", Integral: ");
    Serial.print(integral);
    Serial.print(", Derivative: ");
    Serial.print(derivative);
   // Serial.print(", Kp: ");
   // Serial.print(Kp);
   // Serial.print(", Ki: ");
   // Serial.print(Ki);
   // Serial.print(", Kd: ");
   // Serial.print(Kd);
   Serial.print(", Output: ");
    Serial.println(output);
  }


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
  Ki = Ki_in * speedTimeRate;
  Kd = Kd_in / speedTimeRate;
}

// Serial print of variable resistor values
void debugVarRes(){
  Serial.println("Analog Readings:");
  Serial.print("TickRes: ");
  Serial.println(analogRead(TickRes));
  Serial.print("KpRes: ");
  Serial.println(analogRead(KpRes));
  Serial.print("KiRes: ");
  Serial.println(analogRead(KiRes));  
  Serial.print("KdRes: ");
  Serial.println(analogRead(KdRes));
  Serial.println("~~~~");
}

void setup()
{
  Serial.begin(9600);

  // Motor control pins setup
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // debug reverse encoder directionpin 
  pinMode(PB5,OUTPUT);

  // Analog input pins
  analogReadResolution(PWM_RES);
  pinMode(TickRes, INPUT_ANALOG);
  pinMode(KpRes, INPUT_ANALOG);
  pinMode(KiRes, INPUT_ANALOG);
  pinMode(KdRes, INPUT_ANALOG);

  // Motor control setup
  analogWriteResolution(PWM_RES); // Set PWM resolution
  //analogWrite(ENA, PWM_MAX/2); // Set PWM to 50% duty cycle
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  PIDSetOutputLimits(0, PWM_MAX); // Set output limits for PID controller
  
  // Start speed timer
  SpeedTimer->setOverflow(SPEED_UPDATE_RATE_HZ,HERTZ_FORMAT); // Update speed at the set Hz rate
  SpeedTimer->attachInterrupt(calcTickPerSecSimple); // Attach the speed calculation function to the timer
  SpeedTimer->resume();

  delay(1000);
}

void loop()
{
  debugEncoderReadings();
  delay(100);


  targetTick = map(analogRead(TickRes),0,PWM_MAX,0,3000);
  Serial.print("Target_Tick:");  
  Serial.println(targetTick);
  float Kp_loc = map(analogRead(KpRes),0,PWM_MAX,0,1000);
  float Ki_loc = map(analogRead(KiRes),0,PWM_MAX,0,1000)/1.0;
  float Kd_loc = map(analogRead(KdRes),0,PWM_MAX,0,10);
  //Kd_loc = 0;
  PIDSetTunings(Kp_loc, Ki_loc, Kd_loc);
  Serial.print("Kp:");
  Serial.println(Kp_loc);
  Serial.print("Ki:");
  Serial.println(Ki_loc);
  Serial.print("Kd:");
  Serial.println(Kd_loc);
  //debugVarRes();
}
