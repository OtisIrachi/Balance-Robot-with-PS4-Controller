//**************************************************************************************
// ESP32BRobotPS4Faces.ino
// 
// by RCI
// 6-12-2024 
//
// 11-11-2023
// Added Max7219 LED Eyes.
// 11-12-2023
// Added "void delayms(int loops)"  to allow PID scans during "Blinking Eyes.h" functions.
// 11-14-2023
// Recorded PID - KP = .13,  KD = .08,  PThr = .09,  IThr = 0.0
// 11-18-2023
// Refined Blinking Eyes routine for robot moves.
//
//**************************************************************************************
#include <Wire.h>
#include <Arduino.h>
#include "Control.h"
#include "MPU6050.h"
#include "Motors.h"
#include "defines.h"
#include "globals.h"
#include "BlinkingEyes.h"
#include <Bluepad32.h>

// **** Variables *****
int openEyeFlag;
int LREyeFlag;
int evilEyeFlag;
int eyeProgram;
int servoButtonVal;
unsigned long time_now;

//float BatteryValue;
float BatteryFloat;
char StringBattery[5];
String inputMessage;

float s1faderVal1;
float s1faderVal2;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void initTimers();

//**************************************************************************************
void pidScan() 
{
  timer_value = micros();

  if (MPU6050_newData()) 
    {   
    MPU6050_read_3axis();
    
    dt = (timer_value - timer_old) * 0.000001; // dt in seconds
    //Serial.println(timer_value - timer_old);
    timer_old = timer_value;

    angle_adjusted_Old = angle_adjusted;
    // Get new orientation angle from IMU (MPU6050)
    float MPU_sensor_angle = MPU6050_getAngle(dt);
    angle_adjusted = MPU_sensor_angle + angle_offset;
    if ((MPU_sensor_angle > -15) && (MPU_sensor_angle < 15))
      angle_adjusted_filtered = angle_adjusted_filtered * 0.99 + MPU_sensor_angle * 0.01;

    // We calculate the estimated robot speed:
    // Estimated_Speed = angular_velocity_of_stepper_motors(combined) - angular_velocity_of_robot(angle measured by IMU)
    actual_robot_speed = (speed_M1 + speed_M2) / 2; // Positive: forward

    int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 25.0; // 25 is an empirical extracted factor to adjust for real units
    int16_t estimated_speed = -actual_robot_speed + angular_velocity;
    estimated_speed_filtered = estimated_speed_filtered * 0.9 + (float) estimated_speed * 0.1; // low pass filter on estimated speed

    if (positionControlMode) 
      {
      // POSITION CONTROL. INPUT: Target steps for each motor. Output: motors speed
      motor1_control = positionPDControl(steps1, target_steps1, Kp_position, Kd_position, speed_M1);
      motor2_control = positionPDControl(steps2, target_steps2, Kp_position, Kd_position, speed_M2);

      // Convert from motor position control to throttle / steering commands
      throttle = (motor1_control + motor2_control) / 2;
      throttle = constrain(throttle, -190, 190);
      steering = motor2_control - motor1_control;
      steering = constrain(steering, -50, 50);
      }

    // ROBOT SPEED CONTROL: This is a PI controller.
    //    input:user throttle(robot speed), variable: estimated robot speed, output: target robot angle to get the desired speed
    target_angle = speedPIControl(dt, estimated_speed_filtered, throttle, Kp_thr, Ki_thr);
    target_angle = constrain(target_angle, -max_target_angle, max_target_angle); // limited output

    // Stability control (100Hz loop): This is a PD controller.
    //    input: robot target angle(from SPEED CONTROL), variable: robot angle, output: Motor speed
    //    We integrate the output (sumatory), so the output is really the motor acceleration, not motor speed.
    control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
    control_output = constrain(control_output, -MAX_CONTROL_OUTPUT,  MAX_CONTROL_OUTPUT); // Limit max output from control

    // The steering part from the user is injected directly to the output
    motor1 = control_output + steering;
    motor2 = control_output - steering;

    // Limit max speed (control output)
    motor1 = constrain(motor1, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
    motor2 = constrain(motor2, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

    int angle_ready;
    if (LeftShoulder)     // If we press the SERVO button we start to move
      angle_ready = 82;
    else
      angle_ready = 74;  // Default angle
    if ((angle_adjusted < angle_ready) && (angle_adjusted > -angle_ready)) // Is robot ready (upright?)
      {
      // NORMAL MODE
      digitalWrite(PIN_ENABLE_MOTORS, LOW);  // Motors enable
      // NOW we send the commands to the motors
      setMotorSpeedM1(motor1);
      setMotorSpeedM2(motor2);
      } 
    else   // Robot not ready (flat), angle > angle_ready => ROBOT OFF
      {
      digitalWrite(PIN_ENABLE_MOTORS, HIGH);  // Disable motors
      setMotorSpeedM1(0);
      setMotorSpeedM2(0);
      PID_errorSum = 0;  // Reset PID I term
      Kp = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
      Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;
      // RESET steps
      steps1 = 0;
      steps2 = 0;
      positionControlMode = false;
      throttle = 0;
      steering = 0;
      }
    
    // Push1 Move servo arm
    if (LeftShoulder) 
      {
      if (angle_adjusted > -40)
        ledcWrite(6, SERVO_MAX_PULSEWIDTH);   // was SERVO_MAX_PULSEWIDTH
      else
        ledcWrite(6, SERVO_MIN_PULSEWIDTH);   // was SERVO_MIN_PULSEWIDTH
      } 
    else
      ledcWrite(6, SERVO_AUX_NEUTRO);

    // Normal condition?
    if ((angle_adjusted < 56) && (angle_adjusted > -56)) 
      {
      Kp = Kp_user;            // Default user control gains
      Kd = Kd_user;
      Kp_thr = Kp_thr_user;
      Ki_thr = Ki_thr_user;
      } 
    else // We are in the raise up procedure => we use special control parameters
      {
      Kp = KP_RAISEUP;         // CONTROL GAINS FOR RAISE UP
      Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;
      }

    } // End of new IMU data
     
}// End pidScan
//**************************************************************************************
// Delay (loops) Milliseconds
void delayms(int loops) 
{
    time_now = millis();
    
    while(millis() < time_now + loops)
      {
      pidScan(); 
      delay(1);   
      }
   
}
//**************************************************************************************
void initMPU6050() 
{
  MPU6050_setup();
  delay(500);
  MPU6050_calibrate();
}
//***********************************************************************************************
// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) 
{
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) 
        {
        if (myControllers[i] == nullptr) 
            {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
            }
        }
    if (!foundEmptySlot) 
        {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
        }
}
//***********************************************************************************************
void onDisconnectedController(ControllerPtr ctl) 
{
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) 
        {
        if (myControllers[i] == ctl) 
            {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
            }
        }

    if (!foundController) 
        {  
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
        }
}
//***********************************************************************************************
void processGamepad(ControllerPtr ctl) 
{
    
    if (ctl->l1()) 
       {
       servoButtonVal = ctl->l1();
       LeftShoulder = servoButtonVal;
       QuickBlink(100); 
       }
    else
       {
       servoButtonVal = 0; 
       LeftShoulder = servoButtonVal;
       }
    
    //*******************************************   
    if (ctl->axisRX()) 
       {       
       s1faderVal2 = ctl->axisRX(); 
       JoyStickLftRgt = map(s1faderVal2, 515, -515, -75, 75);
       JoyStickLftRgt = (JoyStickLftRgt / 100) + .5;
       }
    //*******************************************          
    if (ctl->axisRY()) 
       {
       s1faderVal1 = ctl->axisRY();
       JoyStickForRev = map(s1faderVal1, -515, 515, -75, 75);
       JoyStickForRev = (JoyStickForRev / 100) + .5;
       }  
    //******************************************* 

}
//***********************************************************************************************
void getGamepad() 
{
    BP32.update();

    ControllerPtr myController = myControllers[0];

    if (myController && myController->isConnected()) 
       {
       if (myController->isGamepad()) 
          {
          processGamepad(myController);
          }   
       }
 
}
//**************************************************************************************
void setup() 
{
  Serial.begin(115200);         // set up seriamonitor at 115200 bps
  Serial.setDebugOutput(true);
    
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);

  pinMode(PIN_ENABLE_MOTORS, OUTPUT);
  digitalWrite(PIN_ENABLE_MOTORS, HIGH);
  
  pinMode(PIN_MOTOR1_DIR, OUTPUT);
  pinMode(PIN_MOTOR1_STEP, OUTPUT);
  pinMode(PIN_MOTOR2_DIR, OUTPUT);
  pinMode(PIN_MOTOR2_STEP, OUTPUT);
  pinMode(PIN_SERVO, OUTPUT);
  
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  mx.begin();
  mx.control(MD_MAX72XX::INTENSITY, 1);  

  BlankEyes();  

  ledcSetup(6, 50, 16); // channel 6, 50 Hz, 16-bit width
  ledcAttachPin(PIN_SERVO, 6);   // GPIO 22 assigned to channel 1
  delay(50);
  ledcWrite(6, SERVO_AUX_NEUTRO);
  
  Wire.begin();
  initMPU6050();

  initTimers();

  JoyStickForRev = 0.5;
  JoyStickLftRgt = 0.5;
  Kp_user_Preset      = 0.1;
  Kd_user_Preset      = 0.080;
  Kp_thr_user_Preset  = 0.1;
  Ki_thr_user_Preset  = 0.1;
  
  digitalWrite(PIN_ENABLE_MOTORS, LOW);
  for (uint8_t k = 0; k < 5; k++) 
    {
    setMotorSpeedM1(5);
    setMotorSpeedM2(5);
    ledcWrite(6, SERVO_AUX_NEUTRO + 250);
    delay(200);
    setMotorSpeedM1(-5);
    setMotorSpeedM2(-5);
    ledcWrite(6, SERVO_AUX_NEUTRO - 250);
    delay(200);
    }
  ledcWrite(6, SERVO_AUX_NEUTRO);
  eyeProgram = 0;
  openEyeFlag = 1;
  LREyeFlag = 0;
  evilEyeFlag = 0; 
  max_throttle = MAX_THROTTLE;
  max_steering = MAX_STEERING;
  max_target_angle = MAX_TARGET_ANGLE;  

  
}
//**************************************************************************************
void loop() 
{
  
  getGamepad();
  vTaskDelay(1);
  delay(1);  
  processJoysticks();
  pidScan(); 
 
  if((JoyStickForRev == 0.5) || (JoyStickLftRgt == 0.5))   
    {                 
    openEyeFlag = 1;
    }
        
  if(openEyeFlag == 1)
    {
    OpenEyes(1);
    openEyeFlag = 0;
    }

  if(JoyStickForRev < 0.4) 
    {
    EvilEyes(); 
    }

  if(JoyStickForRev > 0.6) 
    {
    RaisedEyes(); 
    }
     
}
//**************************************************************************************
void processJoysticks() 
  {
      
      positionControlMode = false;
      throttle = (JoyStickForRev - 0.5) * max_throttle;

      // We add some exponential on steering to smooth the center b
      steering = JoyStickLftRgt - 0.5;
      if (steering > 0)
        {
        steering = (steering * steering + 0.5 * steering) * max_steering;
        }
      else
        {
        steering = (-steering * steering + 0.5 * steering) * max_steering;
        }
//      }
      
    // User could adjust KP, KD, KP_THROTTLE and KI_THROTTLE (fadder3,4,5,6)
    Kp_user = Kp_user_Preset;
    Kd_user = Kd_user_Preset;
    Kp_thr_user = Kp_thr_user_Preset;
    Ki_thr_user = Ki_thr_user_Preset;


}// End processJoysticks
//**************************************************************************************
