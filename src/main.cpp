#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <stm32yyxx_ll_tim.h>
#include <quadratureDecoder.h>
#include <Adafruit_MCP2515.h>
#include <pidController.h>
#include <motorController.h>

/*
  PID control of a motor using a quadrature encoder for feedback
  Author: Scott Roelker
*/

#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x01090000)
#error "Due to API change, this sketch is compatible with STM32_CORE_VERSION  >= 0x01090000"
#endif

// OLED display 
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 

// Qudrature decoder pin definitions
#define T1CH1_Pin PA8 // Quadrature A channel input pin
#define T1CH2_Pin PA9 // Quadrature B channel input pin

// Motor pin definitions
#define ENA PB8
#define IN1 PB7
#define IN2 PB6
#define ticksPerRevolution 2222 // Number of "ticks" per revolution

// Speed update rate in HZ
#define SPEED_UPDATE_RATE_HZ 20
const float speedTimeRate =  1.0/SPEED_UPDATE_RATE_HZ; // Period of update rate
volatile uint16_t prevTickCnt = 0; // Previously stored Tick Count, initialized to zero upon reset
volatile float ticksPerSec = 0; // Ticks per second of the encoder
int averageRPM = 0; // Average RPM of the motor


// PWM settings
#define PWM_RES 16 // PWM resolution in bits
const int PWM_MAX = (1 << PWM_RES) - 1; // Maximum PWM value

// Variable Resistor Pins
#define TickRes PB1
#define KpRes PB0
#define KiRes PA7
#define KdRes PA6

// Debugging Flags
#define DEBUG_MANUAL_PID 1
#define DEBUG_VAR_RES_RAW 0
#define DEBUG_ENCODER_READINGS 1

void calcTickPerSec();

// Create an QuadratureDecoder object, this wraps around the functionality of the decoder
// The Timer used MUST support Quadrature decoding, otherwise the functions will not work
QuadratureDecoder quadDecoder(TIM1, T1CH1_Pin, T1CH2_Pin);
// Speed timer used to calculate speed and run PID loop
HardwareTimer *SpeedTimer = new HardwareTimer(TIMER_SERVO);
// PID Controller for motor PWM control
PIDController pidController(speedTimeRate, 0, PWM_MAX, 0, 0, 0); 
int targetTick = 0; // Target speed in ticks/second
// Motor Controller initialization
motorController motorControl = motorController(ENA, IN1, IN2,PWM_RES);
motorController::motorDirection motorDir = motorController::FORWARD;
// Use SDA/SCL2 pins
TwoWire Wire2(PB11, PB10); // Create a new Wire instance for the OLED display
// Display Object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET);

// Calculates ticks/second at  a rate of SPEED_UPDATE_HZ
// Use direction as discriminant and account for over/underflow
void calcTickPerSecSimple()
{
  uint32_t curTickCnt = quadDecoder.getCountQuad(TICK_FORMAT);
  bool direction = 0;  
  static int prevDirection = quadDecoder.getDirBit();

  if(prevTickCnt == curTickCnt){
      ticksPerSec = 0;
  }else {
    // Calculate Ticks/Second based on direction

    if(curTickCnt>prevTickCnt){
        ticksPerSec = (curTickCnt - prevTickCnt) / (speedTimeRate);
        direction = 0;
    } else { 
      // Account for overflow
        ticksPerSec = (prevTickCnt - curTickCnt) / (speedTimeRate);
        direction = 1;
    }

    // If we have overflowed, assume the quadrature counter has also overflowed
    // and correct the count to account for this
    if(ticksPerSec>UINT16_MAX){
      if(direction==0){
        ticksPerSec = (prevTickCnt + (UINT16_MAX - curTickCnt - 1)) / (speedTimeRate);
      }else{
        ticksPerSec = (curTickCnt + (UINT16_MAX - prevTickCnt - 1)) / (speedTimeRate);
      }
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
      return;
    }
    
    prevTickCnt = curTickCnt;
  }

  // run PID loop
  int localTargetTick = targetTick;
  // PID controller only takes positive values,
  // so we need to account for the reverse direction
  if(targetTick<0){
    localTargetTick = -targetTick;
    motorControl.setDirection(motorController::BACKWARD);
    motorDir = motorController::BACKWARD;
  }else{
    motorControl.setDirection(motorController::FORWARD);
    motorDir = motorController::FORWARD;
  }
  int motorPWM = pidController.control(localTargetTick, ticksPerSec); 
  motorControl.setPWMOut(motorPWM); // Set motor PWM
}
// Debug print of Encoder readings
void debugEncoderReadings(){
  //Serial.print("Encoder Count:");
  //Serial.println(quadDecoder.getCountQuad(TICK_FORMAT));
  //Serial.print("Direction:");
  //Serial.println(quadDecoder.getDirBit()? "Backward":"Forward");
  //Serial.println(quadDecoder.getDirBit());
  averageRPM = (averageRPM + (ticksPerSec/ticksPerRevolution*60))/2;
  Serial.print("Ticks_per_second:");
  Serial.println(ticksPerSec);
  Serial.print("RPS:");
  Serial.print(ticksPerSec/ticksPerRevolution);
  Serial.print("RPM:");
  Serial.println(ticksPerSec/ticksPerRevolution*60);
  Serial.print("Average RPM:");
  Serial.println(averageRPM);
  //Serial.println("~~~~");
}
// Serial print of variable resistor values
void debugVarResRaw(){
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

void manualPIDControl(){
  targetTick = map(analogRead(TickRes),0,PWM_MAX,-3000,3000);
  float Kp_loc = map(analogRead(KpRes),0,PWM_MAX,0,1000);
  float Ki_loc = map(analogRead(KiRes),0,PWM_MAX,0,1000)/1.0;
  float Kd_loc = map(analogRead(KdRes),0,PWM_MAX,0,10);
  pidController.PIDSetTunings(Kp_loc, Ki_loc, Kd_loc);

  if(DEBUG_MANUAL_PID){
  Serial.print("Target_Tick:");  
  Serial.println(targetTick);
  Serial.print("Kp:");
  Serial.println(Kp_loc);
  Serial.print("Ki:");
  Serial.println(Ki_loc);
  Serial.print("Kd:");
  Serial.println(Kd_loc);
  }
}

void pwmControl(){
  // Only supports forward for now
  motorControl.setDirection(motorController::FORWARD);
  int pwmMotor = map(analogRead(TickRes),0,PWM_MAX,0,PWM_MAX);
  motorControl.setPWMOut(pwmMotor);
}

// Display helper function
void displayControl(){
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("Target TPS: ");;
  display.println(targetTick);  // Print the last part;
  display.print("TPS: ");;
  display.println(ticksPerSec);  // Print the last part;
  display.print("RPS: ");
  display.print(ticksPerSec/ticksPerRevolution);
  display.print(" RPM: ");
  display.println(ticksPerSec/ticksPerRevolution*60);
  display.print("Avg RPM: ");
  display.println(averageRPM);
  display.display();
}

void setup()
{
  Serial.begin(9600);

  // Initialize the OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC,SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.display();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);

  // Analog input pins
  analogReadResolution(PWM_RES);
  pinMode(TickRes, INPUT_ANALOG);
  pinMode(KpRes, INPUT_ANALOG);
  pinMode(KiRes, INPUT_ANALOG);
  pinMode(KdRes, INPUT_ANALOG);

  motorControl.setDirection(motorDir); // Set motor to off
  motorControl.setPWMOut(0); // Set motor PWM to zero

  // PID controller setup and initial values
  pidController.PIDSetOutputLimits(0, PWM_MAX); // Set output limits for PID controller
  float Kp_loc = map(analogRead(KpRes),0,PWM_MAX,0,1000);
  float Ki_loc = map(analogRead(KiRes),0,PWM_MAX,0,1000)/1.0;
  float Kd_loc = map(analogRead(KdRes),0,PWM_MAX,0,10);
  pidController.PIDSetTunings(Kp_loc, Ki_loc, Kd_loc);

  // Start speed timer
  SpeedTimer->setOverflow(SPEED_UPDATE_RATE_HZ,HERTZ_FORMAT); // Update speed at the set Hz rate
  SpeedTimer->attachInterrupt(calcTickPerSecSimple); // Attach the speed calculation function to the timer
  SpeedTimer->resume();

  delay(1000);
}

void loop()
{
  if(DEBUG_VAR_RES_RAW){
    debugVarResRaw();
  }
  if(DEBUG_ENCODER_READINGS){
  debugEncoderReadings();
  }

  // Control PID 
  manualPIDControl();
  
  //pwmControl();
  Serial.print("Target_Tick:");  
  Serial.println(targetTick);
  //targetTick++;
  //if(targetTick>3000){
  //  targetTick = -3000;
  //}

  // Update OLED display
  displayControl();
  
  delay(100);
}
