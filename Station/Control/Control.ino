#include <EasyTransfer.h>

struct JOYSTICK
{
  char ch1[2];
  unsigned int serialn;
  char ch2[2];
  uint8_t forward;
  uint8_t backward;
  uint8_t left;
  uint8_t right;
  uint8_t cw;
  uint8_t ccw;
  uint8_t slider;
  uint8_t hat;
  char ch3[2];
  bool button[13];
  bool chkupd;
};

JOYSTICK Joystick, JoystickStore;

bool chkstorejs = 0;
unsigned int chkcounterjs = 0;

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

CONTROL Control;

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

FEEDBACK Feedback, FeedbackStore;

bool chkstorefb = 0;
unsigned int chkcounterfb = 0;

EasyTransfer JSin;
EasyTransfer ETout;
EasyTransfer ETin;
EasyTransfer MNout;

#define debugBT

#define PWM 1
#define DIR 0

#define Precision 100

bool button4 = 0;
bool button6 = 0;
bool button11 = 0;
bool button12 = 0;

const int motMask[12] = { -1, 1, 1, -1, -1, 1, -1, 1, 1, -1, 1, 1};
const int pitchMask[9] = {1, 0, 0, 0, -1, 0, 0, 0, 0};
const int rollMask[9] = {0, 0, 1, 0, 0, 0, -1, 0, 0};

float headingSet;
float pitchSet;
float rollSet;

float depth;
float depthSet;

char disp;

void motorDrive(int num, int val)
{
  val *= motMask[num];
  if (val >= 0)
  {
    Control.motSet[PWM][num] = val;
    Control.motSet[DIR][num] = 0;
  }
  else
  {
    Control.motSet[PWM][num] = -val;
    Control.motSet[DIR][num] = 1;
  }
  if (Control.motSet[PWM][num] > 255)
    Control.motSet[PWM][num] = 255;
  else if (Control.motSet[PWM][num] >= 0 && Control.motSet[PWM][num] < 50)
    Control.motSet[PWM][num] = 0;
  else if (Control.motSet[PWM][num] >= 50 && Control.motSet[PWM][num] < 60)
    Control.motSet[PWM][num] = 60;
  return;
}

void setup()
{
#ifdef debug
  Serial.begin(230400); // Debug
#endif
#ifdef debugOnboard
  Serial.begin(230400); // Debug
#endif
#ifdef debugBT
  Serial.begin(230400); // Debug
#endif
  Serial1.begin(230400); // Offshore
  Serial2.begin(230400); // Joystick
  Serial3.begin(230400); // Monitor

  memset(&Joystick, 0, sizeof(JOYSTICK));
  memset(&JoystickStore, 0, sizeof(JOYSTICK));
  memset(&Control, 0, sizeof(CONTROL));
  memset(&Feedback, 0, sizeof(FEEDBACK));
  memset(&FeedbackStore, 0, sizeof(FEEDBACK));

  JSin.begin(details(Joystick), &Serial2);
  ETout.begin(details(Control), &Serial1);
  ETin.begin(details(Feedback), &Serial1);
  MNout.begin(details(Feedback), &Serial3);

  Control.ch1[0] = 'R';
  Control.ch2[0] = 'O';
  Control.ch3[0] = 'V';
  Control.ch1[1] = 'R';
  Control.ch2[1] = 'O';
  Control.ch3[1] = 'V';
}

void loop()
{
  JSin.receiveData();

  if (Joystick.ch1[0] != 'J' || Joystick.ch2[0] != 'S' || Joystick.ch3[0] != 'L' || Joystick.ch1[1] != 'J' || Joystick.ch2[1] != 'S' || Joystick.ch3[1] != 'L')
    memcpy(&Joystick, &JoystickStore, sizeof(JOYSTICK));
  else
    memcpy(&JoystickStore, &Joystick, sizeof(JOYSTICK));

  if (Joystick.chkupd == chkstorejs && chkcounterjs > 300)
  {
    memset(&Joystick, 0, sizeof(JOYSTICK));
    memcpy(&JoystickStore, &Joystick, sizeof(JOYSTICK));
    chkstorejs = 0;
  }
  else if (Joystick.chkupd == chkstorejs)
  {
    chkcounterjs++;
  }
  else
  {
    chkcounterjs = 0;
    if (chkstorejs == 0)
      chkstorejs = 1;
    else
      chkstorejs = 0;
  }

  ETin.receiveData();

  if (Feedback.ch1[0] != 'F' || Feedback.ch2[0] != 'B' || Feedback.ch3[0] != 'K' || Feedback.ch1[1] != 'F' || Feedback.ch2[1] != 'B' || Feedback.ch3[1] != 'K')
    memcpy(&Feedback, &FeedbackStore, sizeof(FEEDBACK));
  else
    memcpy(&FeedbackStore, &Feedback, sizeof(FEEDBACK));

  if (Feedback.chkupd == chkstorefb && chkcounterfb > 300)
  {
    memset(&Feedback, 0, sizeof(FEEDBACK));
    memcpy(&FeedbackStore, &Feedback, sizeof(FEEDBACK));
    chkstorefb = 0;
  }
  else if (Feedback.chkupd == chkstorefb)
  {
    chkcounterfb++;
  }
  else
  {
    chkcounterfb = 0;
    if (chkstorefb == 0)
      chkstorefb = 1;
    else
      chkstorefb = 0;
  }

  /////////////////////////////////////////////////////////
  // Printing Debugging Data on the Serial Monitor

#ifdef debug
  Serial.print(" #");
  Serial.println(Joystick.serialn);
  Serial.print(" Forward: ");
  Serial.print(Joystick.forward);
  Serial.print(" \t Backward: ");
  Serial.print(Joystick.backward);
  Serial.print(" \t Left: ");
  Serial.print(Joystick.left);
  Serial.print(" \t Right: ");
  Serial.print(Joystick.right);
  Serial.println("");
  Serial.print(" CW: ");
  Serial.print(Joystick.cw);
  Serial.print(" \t CCW: ");
  Serial.print(Joystick.ccw);
  Serial.println("");
  Serial.print(" Slider: ");
  Serial.print(Joystick.slider);
  Serial.print(" \t Hat: ");
  Serial.print(Joystick.hat);
  Serial.println("");
  Serial.print(" [1]: ");
  Serial.print(Joystick.button[1]);
  Serial.print(" \t [2]: ");
  Serial.print(Joystick.button[2]);
  Serial.print(" \t [3]: ");
  Serial.print(Joystick.button[3]);
  Serial.print(" \t [4]: ");
  Serial.print(Joystick.button[4]);
  Serial.print(" \t [5]: ");
  Serial.print(Joystick.button[5]);
  Serial.print(" \t [6]: ");
  Serial.print(Joystick.button[6]);
  Serial.println("");
  Serial.print(" [7]: ");
  Serial.print(Joystick.button[7]);
  Serial.print(" \t [8]: ");
  Serial.print(Joystick.button[8]);
  Serial.print(" \t [9]: ");
  Serial.print(Joystick.button[9]);
  Serial.print(" \t [10]: ");
  Serial.print(Joystick.button[10]);
  Serial.print(" \t [11]: ");
  Serial.print(Joystick.button[11]);
  Serial.print(" \t [12]: ");
  Serial.print(Joystick.button[12]);
  Serial.println("");
  Serial.println("");
#endif

#ifdef debugOnboard
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

#ifdef debugBT
  if (Serial.available())
  {
    disp = Serial.read();
  }
  if (disp == 'P')
  {
    Serial.print(" IntP ");
    Serial.print(Feedback.ip);
    Serial.print("\t IntTemp ");
    Serial.println(Feedback.it);
  }
  if (disp == 'B')
  {
    if (Feedback.bt != '*')
    {
      Serial.print(Feedback.bt);
      Feedback.bt = '*';
    }
  }
  if (disp == 'M')
  {
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
  }
#endif

  /////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////
  // Stabilization Modes

  depth = Feedback.ep;
  if (!Control.OH_Enable)
  {
    headingSet = Feedback.heading;
    pitchSet = Feedback.pitch;
    rollSet = Feedback.roll;
  }
  if (!Control.DH_Enable)
  {
    depthSet = depth;
  }
  if (Joystick.cw + Joystick.ccw > 0)
    headingSet = Feedback.heading;
  if (Joystick.hat == 0 || Joystick.hat == 4)
    pitchSet = Feedback.pitch;
  if (Joystick.hat == 2 || Joystick.hat == 6)
    rollSet = Feedback.roll;
  if (Joystick.button[5] != Joystick.button[3])
    depthSet = depth;

  /////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////
  // Controlling Surging Thrusters

  if (Joystick.button[1])
  {
    /*
    Joystick.forward/=1.8;
    Joystick.backward/=1.5;
    Joystick.cw/=1.8;
    Joystick.ccw/=1.8;
    Joystick.left/=1.5;
    Joystick.right/=1.5;
    */
    if (Joystick.forward > 100)
      Joystick.forward = Precision;
    else
      Joystick.forward = 0;
    if (Joystick.backward > 100)
      Joystick.backward = Precision;
    else
      Joystick.backward = 0;
    if (Joystick.cw > 100)
      Joystick.cw = Precision;
    else
      Joystick.cw = 0;
    if (Joystick.ccw > 100)
      Joystick.ccw = Precision;
    else
      Joystick.ccw = 0;
    if (Joystick.left > 100)
      Joystick.left = Precision + 80;
    else
      Joystick.left = 0;
    if (Joystick.right > 100)
      Joystick.right = Precision + 80;
    else
      Joystick.right = 0;
  }

  motorDrive(0, Joystick.forward - Joystick.backward + Joystick.cw - Joystick.ccw - Joystick.left + Joystick.right);
  motorDrive(1, (Joystick.forward - Joystick.backward - Joystick.cw + Joystick.ccw + Joystick.left - Joystick.right) * 0.8);
  motorDrive(2, (Joystick.forward - Joystick.backward - Joystick.cw + Joystick.ccw - Joystick.left + Joystick.right) * 0.8);
  motorDrive(3, Joystick.forward - Joystick.backward + Joystick.cw - Joystick.ccw + Joystick.left - Joystick.right);

  motorDrive(8, Joystick.forward - Joystick.backward);
  motorDrive(9, Joystick.forward - Joystick.backward);

  /////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////
  // Controlling Heaving Thrusters

  motorDrive(4, Joystick.slider * Joystick.button[5] - Joystick.slider * Joystick.button[3] - Joystick.slider * pitchMask[Joystick.hat] + Joystick.slider * rollMask[Joystick.hat]);
  motorDrive(5, Joystick.slider * Joystick.button[5] - Joystick.slider * Joystick.button[3] - Joystick.slider * pitchMask[Joystick.hat] - Joystick.slider * rollMask[Joystick.hat]);
  motorDrive(6, Joystick.slider * Joystick.button[5] - Joystick.slider * Joystick.button[3] + Joystick.slider * pitchMask[Joystick.hat] - Joystick.slider * rollMask[Joystick.hat]);
  motorDrive(7, Joystick.slider * Joystick.button[5] - Joystick.slider * Joystick.button[3] + Joystick.slider * pitchMask[Joystick.hat] + Joystick.slider * rollMask[Joystick.hat]);

  /////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////
  // Controlling Gripper and Valve-opener

  if (Joystick.button[11] == 1 && Joystick.button[12] == 0) // Opening Gripper
  {
    Control.motSet[PWM][11] = 180;
    Control.motSet[DIR][11] = 0;
  }
  else if (Joystick.button[11] == 0 && Joystick.button[12] == 1) // Closing Gripper
  {
    Control.motSet[PWM][11] = 180;
    Control.motSet[DIR][11] = 1;
  }
  else
  {
    Control.motSet[PWM][11] = 0;
    Control.motSet[DIR][11] = 0;
  }

  if (Joystick.button[9] == 1 && Joystick.button[10] == 0) // Opening Valve (( CCW ))
  {
    Control.motSet[PWM][10] = 160;
    Control.motSet[DIR][10] = 0;
  }
  else if (Joystick.button[9] == 0 && Joystick.button[10] == 1) // Closing Valve (( CW ))
  {
    Control.motSet[PWM][10] = 160;
    Control.motSet[DIR][10] = 1;
  }
  else
  {
    Control.motSet[PWM][10] = 0;
    Control.motSet[DIR][10] = 0;
  }

  /////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////
  // Switching the LED White and Red Lights

  if (Joystick.button[6] == 1 && button6 == 0)
  {
    if (Control.LED_W == 0)
      Control.LED_W = 1;
    else
      Control.LED_W = 0;
    button6 = 1;
  }
  else
  {
    button6 = Joystick.button[6];
  }

  if (Joystick.button[4] == 1 && button4 == 0)
  {
    if (Control.LED_R == 0)
      Control.LED_R = 1;
    else
      Control.LED_R = 0;
    button4 = 1;
  }
  else
  {
    button4 = Joystick.button[4];
  }

  /////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////
  // Enabling and Disabling the Stabilization Modes

  if (Joystick.button[11] == 1 && button11 == 0) // Enabling Orientation Hold Mode
  {
    if (Control.OH_Enable == 0)
      Control.OH_Enable = 1;
    else
      Control.OH_Enable = 0;
    button11 = 1;
  }
  else
  {
    button11 = Joystick.button[11];
  }

  if (Joystick.button[12] == 1 && button12 == 0) // Enabling Depth Hold Mode
  {
    if (Control.DH_Enable == 0)
      Control.DH_Enable = 1;
    else
      Control.DH_Enable = 0;
    button12 = 1;
  }
  else
  {
    button12 = Joystick.button[12];
  }

  /////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////
  // Toggling the Communication Checker Boolean

  if (Control.chkupd == 0)
    Control.chkupd = 1;
  else
    Control.chkupd = 0;

  /////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////
  // Sending Motors' Signals to the Offshore Controller

  ETout.sendData();
  MNout.sendData();

  /////////////////////////////////////////////////////////
}
