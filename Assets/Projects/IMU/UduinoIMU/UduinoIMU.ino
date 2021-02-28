#include "Uduino.h"  // Include Uduino library at the top of the sketch
Uduino uduino("IMU");

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


MPU6050 mpu(0x69); // <-- use for AD0 high, otherwise 0x68 (i2c scanner will give it to you)

// MPU control/status vars
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)

  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  Serial.begin(38400);

  while (!Serial); // wait for Leonardo enumeration, others continue immediately
//  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  //Serial.println(F("Testing device connections..."));
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  while (Serial.available() && Serial.read()); // empty buffer again
  delay(1000);
 // Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(54); //++
  mpu.setYGyroOffset(-21); //--
  mpu.setZGyroOffset(5);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
   // Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
   // Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    // Serial.println(F(")..."));

    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
 //   Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // Error
    Serial.println("Error!");
  }

}



void loop() {
  uduino.update();

  if (uduino.isInit()) {
    if (!dmpReady) {
      uduino.println("IMU not connected.");
      delay(10);
      return;
    }
 
    int  mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) { // check if overflow
      mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;

      SendQuaternion();
      //SendEuler();
      //SendYawPitchRoll();
      //SendRealAccel();
      //SendWorldAccel();
    }
  }
}



void SendQuaternion() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  uduino.print("r/");
  uduino.print(q.w, 4); uduino.print("/");
  uduino.print(q.x, 4); uduino.print("/");
  uduino.print(q.y, 4); uduino.print("/");
  uduino.println(q.z, 4);
}

void SendEuler() {
  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetEuler(euler, &q);
  uduino.print(euler[0] * 180 / M_PI); uduino.print("/");
  uduino.print(euler[1] * 180 / M_PI); uduino.print("/");
  uduino.println(euler[2] * 180 / M_PI);
}

void SendYawPitchRoll() {
  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  uduino.print(ypr[0] * 180 / M_PI); uduino.print("/");
  uduino.print(ypr[1] * 180 / M_PI); uduino.print("/");
  uduino.println(ypr[2] * 180 / M_PI);
}

void SendRealAccel() {
  // display real acceleration, adjusted to remove gravity
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  uduino.print("a/");
  uduino.print(aaReal.x); uduino.print("/");
  uduino.print(aaReal.y); uduino.print("/");
  uduino.println(aaReal.z);
}

void SendWorldAccel() {
  // display initial world-frame acceleration, adjusted to remove gravity
  // and rotated based on known orientation from quaternion
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  uduino.print("a/");
  uduino.print(aaWorld.x); uduino.print("/");
  uduino.print(aaWorld.y); uduino.print("/");
  uduino.println(aaWorld.z);
}
