#include <EasyTransfer.h>
#include "I2Cdev.h"
#include "MPU6050_9Axis_MotionApps41.h"
#include <SFE_BMP180.h>
#include <SparkFun_MS5803_I2C.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define HMC5883L_DEFAULT_ADDRESS    0x1E
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAZ_H         0x05
#define HMC5883L_RA_DATAY_H         0x07

#define debug
#define SoftStart
//#define IMU
#define InternalSensor
//#define ExternalSensor
#define BT

#define PWM 1
#define DIR 0

#define IndLED1 34 // Yellow
#define IndLED2 40 // Green

#define RelCam 35
#define Rel1 41
#define Rel2 43

struct CONTROL
{
  char ch1[2];
  uint8_t motSet[2][12];
  char ch2[2];
  bool LED_W;
  bool LED_R;
  bool OH_Enable;
  bool DH_Enable;
  char ch3[2];
  bool chkupd;
};

CONTROL Control, ControlStore;

struct FEEDBACK
{
  char ch1[2];
  int pitch;
  int roll;
  int heading;
  char ch2[2];
  double ip;
  double it;
  double ep;
  double et;
  char ch3[2];
  char bt;
  bool LED_W;
  bool LED_R;
  bool OH_Enable;
  bool DH_Enable;
  bool chkupd;
};

FEEDBACK Feedback;

EasyTransfer ETin;
EasyTransfer ETout;

MPU6050 mpu;
SFE_BMP180 Internal;

MS5803 External(ADDRESS_HIGH);

const int motPWM[12] = {5, 11, 13, 7, 4, 10, 12, 6, 9, 3, 8, 44};
const int motDir[12] = {A8, A2, A0, A6, A9, A3, A1, A7, A4, A10, A5, A11};

int motCur[2][12] = {0};

bool chkstore = 0;
unsigned int chkcounter = 0;

double P = 0, T = 0;
int status = 0, status2 = 1, lp = 0;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
int16_t mx, my, mz;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];
float heading;

double mxMin = 0;
double mxMax = 308.0;
double myMin = -140.0;
double myMax = 0;
double mzMin = -258.0;
double mzMax = 0;
double mxOffSet = 154.0;
double myOffSet = -70.0;
double mzOffSet = -129.0;

void motorWrite(int num, int val, int dir)
{
  if (dir != motCur[DIR][num])
  {
    analogWrite(motPWM[num], 0);
    digitalWrite(motDir[num], dir);
    motCur[PWM][num] = 0;
    motCur[DIR][num] = dir;
#ifdef SoftStart
    delay(motCur[PWM][num] * 0.04);
#endif
  }
  if (val >= motCur[PWM][num])
  {
    for (int i = motCur[PWM][num]; i < val; i += 20)
    {
      if (i > val)
        i = val;
      analogWrite(motPWM[num], i);
#ifdef SoftStart
      delay(1);
#endif
    }
  }
  else
  {
    analogWrite(motPWM[num], val);
  }
  motCur[PWM][num] = val;
  return;
}

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup()
{
  pinMode(IndLED1, OUTPUT);
  pinMode(IndLED2, OUTPUT);
  digitalWrite(IndLED1, HIGH);
  digitalWrite(IndLED2, LOW);

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24;
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  mpu.initialize();

#ifdef debug
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
#endif

  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  mpu.setI2CMasterModeEnabled(0);
  mpu.setI2CBypassEnabled(1);

  Wire.beginTransmission(HMC5883L_DEFAULT_ADDRESS);
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(HMC5883L_DEFAULT_ADDRESS);
  Wire.write(0x00);
  Wire.write(B00011000);
  Wire.endTransmission();
  delay(5);

  mpu.setI2CBypassEnabled(0);

  mpu.setSlaveAddress(0, HMC5883L_DEFAULT_ADDRESS | 0x80);
  mpu.setSlaveRegister(0, HMC5883L_RA_DATAX_H);
  mpu.setSlaveEnabled(0, true);
  mpu.setSlaveWordByteSwap(0, false);
  mpu.setSlaveWriteMode(0, false);
  mpu.setSlaveWordGroupOffset(0, false);
  mpu.setSlaveDataLength(0, 2);

  mpu.setSlaveAddress(1, HMC5883L_DEFAULT_ADDRESS | 0x80);
  mpu.setSlaveRegister(1, HMC5883L_RA_DATAY_H);
  mpu.setSlaveEnabled(1, true);
  mpu.setSlaveWordByteSwap(1, false);
  mpu.setSlaveWriteMode(1, false);
  mpu.setSlaveWordGroupOffset(1, false);
  mpu.setSlaveDataLength(1, 2);

  mpu.setSlaveAddress(2, HMC5883L_DEFAULT_ADDRESS | 0x80);
  mpu.setSlaveRegister(2, HMC5883L_RA_DATAZ_H);
  mpu.setSlaveEnabled(2, true);
  mpu.setSlaveWordByteSwap(2, false);
  mpu.setSlaveWriteMode(2, false);
  mpu.setSlaveWordGroupOffset(2, false);
  mpu.setSlaveDataLength(2, 2);

  mpu.setI2CMasterModeEnabled(1);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
#ifdef debug
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
#endif
  }

#ifdef debug
  Serial.begin(230400); // Debug
#endif
  Serial1.begin(230400); // Station
  Serial2.begin(9600); // Bluetooth

  memset(&Control, 0, sizeof(CONTROL));
  memset(&ControlStore, 0, sizeof(CONTROL));
  memset(&Feedback, 0, sizeof(FEEDBACK));

  pinMode(RelCam, OUTPUT);
  pinMode(Rel1, OUTPUT);
  pinMode(Rel2, OUTPUT);
  digitalWrite(RelCam, LOW);
  digitalWrite(Rel1, LOW);
  digitalWrite(Rel2, LOW);

  for (int i = 0; i < 12; i++)
  {
    pinMode(motDir[i], OUTPUT);
    pinMode(motPWM[i], OUTPUT);
    motorWrite(i, 0, 0);
  }

  Internal.begin();
  External.reset();
  External.begin();
  ETin.begin(details(Control), &Serial1);
  ETout.begin(details(Feedback), &Serial1);

  digitalWrite(IndLED2, HIGH);

  Feedback.ch1[0] = 'F';
  Feedback.ch2[0] = 'B';
  Feedback.ch3[0] = 'K';
  Feedback.ch1[1] = 'F';
  Feedback.ch2[1] = 'B';
  Feedback.ch3[1] = 'K';
}

void loop()
{
  ETin.receiveData();

  if (Control.ch1[0] != 'R' || Control.ch2[0] != 'O' || Control.ch3[0] != 'V' || Control.ch1[0] != 'R' || Control.ch2[0] != 'O' || Control.ch3[0] != 'V')
    memcpy(&Control, &ControlStore, sizeof(CONTROL));
  else
    memcpy(&ControlStore, &Control, sizeof(CONTROL));

  if (Control.chkupd == chkstore && chkcounter > 300)
  {
    memset(&Control, 0, sizeof(CONTROL));
    memcpy(&Control, &ControlStore, sizeof(CONTROL));
    chkstore = 0;
    digitalWrite(IndLED1, LOW);
  }
  else if (Control.chkupd == chkstore)
  {
    chkcounter++;
    digitalWrite(IndLED1, HIGH);
  }
  else
  {
    chkcounter = 0;
    digitalWrite(IndLED1, HIGH);
    if (chkstore == 0)
      chkstore = 1;
    else
      chkstore = 0;
  }

  for (int i = 0; i < 12; i++)
    motorWrite(i, Control.motSet[PWM][i], Control.motSet[DIR][i]);

#ifdef debug
  for (int i = 0; i < 12; i++)
  {
    if (!i)
      Serial.println("\n Motors Signals");
    Serial.print(" #");
    Serial.print(i);
    Serial.print(":\t");
    Serial.print(Control.motSet[PWM][i]);
    Serial.print(" ");
    Serial.print(Control.motSet[DIR][i]);
    if (i == 3 || i == 7 || i == 11)
      Serial.print("\n");
    else
      Serial.print("\t");
  }

  Serial.println("\n Lights States");
  Serial.print(" White: ");
  if (Control.LED_W)
    Serial.print("ON\t");
  else
    Serial.print("OFF\t");
  Serial.print(" Red: ");
  if (Control.LED_R)
    Serial.print("ON\n");
  else
    Serial.print("OFF\n");

  Serial.println("\n Feedback");
  Serial.print(" Heading ");
  Serial.print(Feedback.heading);
  Serial.print("\t Pitch ");
  Serial.print(Feedback.pitch);
  Serial.print("\t Roll ");
  Serial.println(Feedback.roll);
  Serial.print(" IntP ");
  Serial.print(Feedback.ip);
  Serial.print("\t ExtP ");
  Serial.print(Feedback.ep);
  Serial.print("\t IntTemp ");
  Serial.print(Feedback.it);
  Serial.print("\t ExtTemp ");
  Serial.print(Feedback.et);
  Serial.print("\t BT ");
  Serial.println(Feedback.bt);
#endif

  digitalWrite(Rel1, Control.LED_W);
  digitalWrite(Rel2, Control.LED_R);

  Feedback.LED_W = Control.LED_W;
  Feedback.LED_R = Control.LED_R;

  Feedback.OH_Enable = Control.OH_Enable;
  Feedback.DH_Enable = Control.DH_Enable;

  if (Serial2.available())
  {
    Feedback.bt = Serial2.read();
  }
  else
  {
    Feedback.bt = '*';
  }

#ifdef IMU
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
  }
  else if (mpuIntStatus & 0x02)
  {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    mx = mpu.getExternalSensorWord(0);
    my = mpu.getExternalSensorWord(2);
    mz = mpu.getExternalSensorWord(4);

    double Xh = mx * cos(ypr[1]) + my * sin(ypr[2]) * sin(ypr[1]) - mz * cos(ypr[2]) * sin(ypr[1]);
    double Yh = my * cos(ypr[2]) - mz * sin(ypr[2]);
    float heading = atan2 (Yh, Xh); //Yh Xh
    if (heading < 0) heading += 2 * PI;
    if (heading > 2 * PI) heading -= 2 * PI;
    float headingDegrees = heading * 180 / M_PI;

    Feedback.pitch = ypr[1] * 180 / M_PI;
    Feedback.roll = ypr[2] * 180 / M_PI;
    Feedback.heading = headingDegrees;

  }
#endif

#ifdef InternalSensor
  if (status == 0)
  {
    if (status2 == 1) {if (lp) {Internal.getTemperature(T); status = 1;} else status = 2 + Internal.startTemperature(); }
    if (status2 == 0) {if (lp) {Internal.getPressure(P, T); status = 1;} else status = 2 + Internal.startPressure(3); }
    lp++;
    if (lp == 2)
    {
      status2 = !status2;
      lp = 0;
    }
  }

  status --;
#endif

  Feedback.it = T;
  Feedback.ip = P;

#ifdef ExternalSensor
  Feedback.et = External.getTemperature(CELSIUS, ADC_512);
  Feedback.ep = External.getPressure(ADC_4096);
#endif


  /////////////////////////////////////////////////////////
  // Toggling the Communication Checker Boolean

  if (Feedback.chkupd == 0)
    Feedback.chkupd = 1;
  else
    Feedback.chkupd = 0;

  /////////////////////////////////////////////////////////

  ETout.sendData();
}

