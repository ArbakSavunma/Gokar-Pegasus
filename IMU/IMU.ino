#define INTERRUPT_PIN 2
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;


// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer
int16_t * first_sets;
int16_t * second_sets;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

struct Orientation {
  double Yaw;
  double Pitch;
  double Roll;
  bool Error;
};

struct Orientation prevOrientation;

void initializeIMU() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  devStatus = mpu.dmpInitialize();

  
  mpu.setXAccelOffset(1356.00000);
  mpu.setYAccelOffset(533.00000);
  mpu.setZAccelOffset(1702.00000);
  mpu.setXGyroOffset(48);
  mpu.setYGyroOffset(-11.00000);
  mpu.setZGyroOffset(25.00000);
//1356.00000,	533.00000,	1702.00000,	48.00000,	-11.00000,	25.00000
//-1012.00000,  1285.00000, 1322.00000, 76.00000, -36.00000,  14.00000
//-560.00000,  -2681.00000,  1680.00000, 35.00000, 75.00000, -39.00000


  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
  }
}

struct Orientation getIMUOrientation() {
  if (!dmpReady || !mpu.testConnection()) {
    struct Orientation o;
    o.Yaw = 0;
    o.Pitch = 0;
    o.Roll = 0;
    o.Error = true;
    return o;
  }

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    struct Orientation o;
    o.Yaw = ypr[0] * 180 / M_PI;
    o.Pitch = ypr[2] * 180 / M_PI;
    o.Roll = ypr[1] * 180 / M_PI;
    o.Error = false;

    prevOrientation.Yaw = o.Yaw;
    prevOrientation.Pitch = o.Pitch;
    prevOrientation.Roll = o.Roll;

    return o;
  } else {
    return prevOrientation;
  }
}
