#include "I2Cdev.h"  
#include "MPU6050_6Axis_MotionApps20.h"

// ================================================================
// ===                      INITILIZATIONS                      ===
// ================================================================


//---- Defining Motor Control Pins on Arduino
int pin_fwd = 7;                //-- IN1 - Motor 1                               
int pin_bwd = 6;                //-- IN2 - Motor 1
int pin_fwd2 = 5;               //-- IN3 - Motor 2
int pin_bwd2 =  4;              //-- IN4 - Motor 2 

int pin_pwm1 = 9;               //-- ENA - Motor 1
int pin_pwm2 = 10;              //-- ENB - Motor 2

int motorPWM1 = 0;             //-- PWM Value Motor 1
int motorPWM2 = 0;             //-- PWM Value, Motor 2

int Kp = 2.83;                    //-- Proportional Controller Constant
int Error = 0;

//---- I2C Library for using Wire 
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      #include "Wire.h"
#endif

MPU6050 mpu;                   //-- Creating Sensor Object

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

float correct;
int j = 0;
bool blinkState = false;

//---- MPU control status vars
bool dmpReady = false;          //-- set true if DMP init was successful
uint8_t mpuIntStatus;           //-- holds actual interrupt status byte from MPU
uint8_t devStatus;              //-- return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;            //-- expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;             //-- count of all bytes currently in FIFO
uint8_t fifoBuffer[64];         //-- FIFO storage buffer

// orientation-motion vars
Quaternion q;                   //-- [w, x, y, z]         quaternion container
VectorInt16 aa;                 //-- [x, y, z]            accel sensor measurements
VectorInt16 aaReal;             //-- [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;            //-- [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;            //-- [x, y, z]            gravity vector
float euler[3];                 //-- [psi, theta, phi]    Euler angle container
float ypr[3];                   //-- [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  pinMode(pin_fwd, OUTPUT);
  pinMode(pin_bwd, OUTPUT);
  pinMode(pin_fwd2, OUTPUT);
  pinMode(pin_bwd2, OUTPUT);

  //---- Prepare IC2 for use on Arduino 
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);    //-- 400kHz I2C Clock
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  //---- Initiliing Serial Communication
  Serial.begin(115200);                                        
  while (!Serial); 

  //---- Initializing Sensor Communication
     Serial.println(F("Initializing I2C devices..."));
     mpu.initialize();
     pinMode(INTERRUPT_PIN, INPUT);
     devStatus = mpu.dmpInitialize();
       
     //-- Gyro Offset Values Settings
     mpu.setXGyroOffset(17);
     mpu.setYGyroOffset(-69);
     mpu.setZGyroOffset(27);
     mpu.setZAccelOffset(1551);
      
     //-- Confirmation Check
     if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        //-- Attaching Interrupt to obtain information
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.println("Initialization");
      } 
      else { Serial.println("Initialization failed");  }
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  //---- If Initilization Failed. Return & End
  if (!dmpReady) return;



  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      // try to get out of the infinite loop
      fifoCount = mpu.getFIFOCount();
    }
  }
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;



    //---- Data from Gyroscope: Yaw, Pitch and Roll values
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    //---- Pitch, Roll values - Radians to degrees
    ypr[1] = ypr[1] * 180 / M_PI;
    ypr[2] = ypr[2] * 180 / M_PI;

    //---- Some additional adjustments
    ypr[2] = -1*ypr[2] -1 ;
    ypr[1] = ypr[1]-3;
    }
    else {
 
       //---- Implementing Proportional Controller
      if (ypr[2] >= 0){                     //---- If +ve 90 w.r.t to x-axis
          Error = abs(ypr[2])
          digitalWrite(pin_fwd2, LOW);
          digitalWrite(pin_bwd2, HIGH);
          motorPWM2 = 35 + (Error*Kp);
          analogWrite(pin_pwm2, motorPWM2); 
          }    
      else if (ypr[2] < 0){                 //---- If  -ve 90 w.r.t to x-axis
          Error = abs(ypr[2])
          digitalWrite(pin_fwd2, HIGH);
          digitalWrite(pin_bwd2, LOW);
          motorPWM2 = 60 + (Error*Kp);
          analogWrite(pin_pwm2, motorPWM2);  
          }
      else{ ypr[2] = 0;   }             
      

      if (ypr[1] >= 0){                     //---- If +ve 90 w.r.t to y-axis
          Error = abs(ypr[1])
          digitalWrite(pin_fwd, HIGH);
          digitalWrite(pin_bwd, LOW);
          motorPWM1 = 35 + (Error*Kp);
          analogWrite(pin_pwm1, motorPWM1);
          }
      else if (ypr[1] < 0){                 //---- If -ve 90 w.r.t to y-axis
          digitalWrite(pin_fwd, LOW);
          digitalWrite(pin_bwd, HIGH);
          motorPWM1 = 55 + (Error*Kp);
          analogWrite(pin_pwm1, motorPWM1);
          }
      else { ypr[1] = 0;  }

      //---- Printing Actuating Signal and Angle Values from Gyro.      
      Serial.print("MotorX: ");      Serial.print(motorPWM2);      Serial.print (" | ");
      Serial.print("MotorY: ");      Serial.print(motorPWM1);      Serial.print(" | ");
      Serial.print("AngleX: ");      Serial.print(ypr[2]);         Serial.print(" | ");
      Serial.print("AngleY: ");      Serial.print(ypr[1]);         Serial.println(" ");      
    }
    
#endif
  }
}
