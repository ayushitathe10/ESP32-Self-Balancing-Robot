/*
 * ESP32 Self-Balancing Robot
 *
 * MPU6050 + DMP + PID + TB6612FNG
 *
 * Based on the original Arduino Uno code.
 *
 * Control:
 * MPU6050 -> Pitch angle -> PID -> Motor PWM
 *
 * Hardware:
 * ESP32
 * MPU6050
 * TB6612FNG
 * 2 x DC geared motors
 */

#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"

// MPU6050


MPU6050 mpu;

// MPU control/status variables
bool dmpReady = false;

uint8_t mpuIntStatus;
uint8_t devStatus;

uint16_t packetSize;
uint16_t fifoCount;

uint8_t fifoBuffer[64];

// Orientation variables
Quaternion q;
VectorFloat gravity;

float ypr[3];

// MPU6050 INTERRUPT PIN


#define MPU_INT_PIN 27

volatile bool mpuInterrupt = false;


// ISR
void IRAM_ATTR dmpDataReady()
{
    mpuInterrupt = true;
}

// PID SETTINGS

// Original Arduino values
double setpoint = 176;

double Kp = 21;
double Kd = 0.8;
double Ki = 140;

double input;
double output;

// PID object
PID pid(
    &input,
    &output,
    &setpoint,
    Kp,
    Ki,
    Kd,
    DIRECT
);

// TB6612FNG PIN DEFINITIONS

// Left motor
#define AIN1 25
#define AIN2 26
#define PWMA 14

// Right motor
#define BIN1 32
#define BIN2 33
#define PWMB 13

// Standby
#define STBY 12


// ESP32 PWM SETTINGS


#define PWM_FREQ 20000
#define PWM_RESOLUTION 8

#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1

// MOTOR FUNCTIONS


void Forward();
void Reverse();
void Stop();

// SETUP

void setup()
{
    Serial.begin(115200);

   
    // I2C
   

    Wire.begin(21, 22);

    Wire.setClock(400000);


    
    // Motor pins
    

    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);

    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    pinMode(STBY, OUTPUT);


    // Enable TB6612FNG
    digitalWrite(STBY, HIGH);


    
    // ESP32 PWM
    

    ledcSetup(
        PWM_CHANNEL_A,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcSetup(
        PWM_CHANNEL_B,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcAttachPin(PWMA, PWM_CHANNEL_A);
    ledcAttachPin(PWMB, PWM_CHANNEL_B);


    // Initially stop motors
    Stop();


    
    // MPU6050 initialization
    

    Serial.println(
        "Initializing I2C devices..."
    );

    mpu.initialize();


    // Check MPU6050 connection
    Serial.println(
        "Testing device connections..."
    );

    if (mpu.testConnection())
    {
        Serial.println(
            "MPU6050 connection successful"
        );
    }
    else
    {
        Serial.println(
            "MPU6050 connection failed"
        );

        while (1)
        {
            delay(100);
        }
    }


    
    // Initialize DMP
    

    Serial.println(
        "Initializing DMP..."
    );

    devStatus = mpu.dmpInitialize();


    
    // Gyroscope and accelerometer offsets
    

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);

    mpu.setZAccelOffset(1688);


    
    // Check DMP initialization
    

    if (devStatus == 0)
    {
        Serial.println(
            "DMP initialization successful"
        );


        // Enable DMP
        mpu.setDMPEnabled(true);


        
        // Enable MPU interrupt
        

        pinMode(
            MPU_INT_PIN,
            INPUT
        );

        attachInterrupt(
            digitalPinToInterrupt(MPU_INT_PIN),
            dmpDataReady,
            RISING
        );


        // Get interrupt status
        mpuIntStatus = mpu.getIntStatus();


        // DMP is ready
        dmpReady = true;


        // Get expected packet size
        packetSize =
            mpu.dmpGetFIFOPacketSize();


        Serial.println(
            "DMP ready! Waiting for first interrupt..."
        );


        
        // PID setup
        

        pid.SetMode(AUTOMATIC);

        // Same as original code
        pid.SetSampleTime(10);

        // Motor PWM range
        pid.SetOutputLimits(-255, 255);


        Serial.println(
            "PID initialized"
        );
    }

    else
    {
        Serial.print(
            "DMP Initialization failed (code "
        );

        Serial.print(devStatus);

        Serial.println(")");
    }
}



// MAIN LOOP


void loop()
{
    // If DMP initialization failed
    if (!dmpReady)
    {
        return;
    }


    
    // Wait for MPU interrupt or available packet
    

    while (
        !mpuInterrupt &&
        fifoCount < packetSize
    )
    {

        
        // PID calculation
        

        pid.Compute();


        // Debug
        Serial.print(input);
        Serial.print(" => ");
        Serial.println(output);


        
        // Check robot angle
        

        if (
            input > 150 &&
            input < 200
        )
        {
            // Robot is in balancing range


            if (output > 0)
            {
                // Falling toward front

                Forward();
            }

            else if (output < 0)
            {
                // Falling toward back

                Reverse();
            }

            else
            {
                Stop();
            }
        }

        else
        {
            // Robot has fallen
            Stop();
        }
    }


    
    // Reset interrupt flag
    

    mpuInterrupt = false;


    // Get interrupt status
    mpuIntStatus =
        mpu.getIntStatus();


    // Get FIFO count
    fifoCount =
        mpu.getFIFOCount();


    
    // Check FIFO overflow
    

    if (
        (mpuIntStatus & 0x10) ||
        fifoCount == 1024
    )
    {
        mpu.resetFIFO();

        Serial.println(
            "FIFO overflow!"
        );
    }


    
    // Check if DMP data is ready
    

    else if (
        mpuIntStatus & 0x02
    )
    {

        // Wait until complete packet available

        while (
            fifoCount < packetSize
        )
        {
            fifoCount =
                mpu.getFIFOCount();
        }


        // Read DMP packet

        mpu.getFIFOBytes(
            fifoBuffer,
            packetSize
        );


        // Update FIFO count

        fifoCount -= packetSize;


        
        // Convert DMP data
        

        mpu.dmpGetQuaternion(
            &q,
            fifoBuffer
        );


        mpu.dmpGetGravity(
            &gravity,
            &q
        );


        mpu.dmpGetYawPitchRoll(
            ypr,
            &q,
            &gravity
        );


        
        // Calculate pitch angle
        

        input =
            ypr[1] * 180 / M_PI + 180;


        // Debug angle
        Serial.print("Angle: ");
        Serial.println(input);
    }
}



// FORWARD


void Forward()
{
    int pwm =
        constrain(
            abs((int)output),
            0,
            255
        );


    // Left motor forward
    digitalWrite(
        AIN1,
        HIGH
    );

    digitalWrite(
        AIN2,
        LOW
    );


    // Right motor forward
    digitalWrite(
        BIN1,
        HIGH
    );

    digitalWrite(
        BIN2,
        LOW
    );


    // PWM
    ledcWrite(
        PWM_CHANNEL_A,
        pwm
    );

    ledcWrite(
        PWM_CHANNEL_B,
        pwm
    );


    Serial.println("F");
}



// REVERSE


void Reverse()
{
    int pwm =
        constrain(
            abs((int)output),
            0,
            255
        );


    // Left motor reverse
    digitalWrite(
        AIN1,
        LOW
    );

    digitalWrite(
        AIN2,
        HIGH
    );


    // Right motor reverse
    digitalWrite(
        BIN1,
        LOW
    );

    digitalWrite(
        BIN2,
        HIGH
    );


    // PWM
    ledcWrite(
        PWM_CHANNEL_A,
        pwm
    );

    ledcWrite(
        PWM_CHANNEL_B,
        pwm
    );


    Serial.println("R");
}



// STOP


void Stop()
{
    digitalWrite(
        AIN1,
        LOW
    );

    digitalWrite(
        AIN2,
        LOW
    );


    digitalWrite(
        BIN1,
        LOW
    );

    digitalWrite(
        BIN2,
        LOW
    );


    ledcWrite(
        PWM_CHANNEL_A,
        0
    );

    ledcWrite(
        PWM_CHANNEL_B,
        0
    );


    Serial.println("S");
}