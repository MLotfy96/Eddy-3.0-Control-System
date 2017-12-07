#include <EasyTransfer.h>
#if !defined(__HIDJOYSTICKRPTPARSER_H__)
#define __HIDJOYSTICKRPTPARSER_H__

EasyTransfer JSout;

#define debug

#include <hid.h>
#include <hiduniversal.h>
#include <usbhub.h>

// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

struct GamePadEventData
{
  union { //axes and hut switch
    uint32_t axes;
    struct {
      uint32_t x : 10;
      uint32_t y : 10;
      uint32_t hat : 4;
      uint32_t twist : 8;
    };
  };
  uint8_t buttons_a;
  uint8_t slider;
  uint8_t buttons_b;
};

class JoystickEvents
{
public:
  virtual void OnGamePadChanged(const GamePadEventData *evt);
};

#define RPT_GAMEPAD_LEN sizeof(GamePadEventData)/sizeof(uint8_t)

class JoystickReportParser : public HIDReportParser
{
  JoystickEvents    *joyEvents;

  uint8_t oldPad[RPT_GAMEPAD_LEN];

public:
  JoystickReportParser(JoystickEvents *evt);

  virtual void Parse(HID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf);
};


USB                                             Usb;
USBHub                                          Hub(&Usb);
HIDUniversal                                    Hid(&Usb);
JoystickEvents                                  JoyEvents;
JoystickReportParser                            Joy(&JoyEvents);

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

JOYSTICK Joystick;

bool chksend;

void setup()
{
#ifdef debug
  Serial.begin(230400); // Debug
#endif
  Serial1.begin(230400); // Station

  memset(&Joystick, 0, sizeof(JOYSTICK));

  JSout.begin(details(Joystick), &Serial1);

#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
#ifdef debug
  Serial.println("Start");

  if (Usb.Init() == -1)
    Serial.println("OSC did not start.");
#endif
  delay(200);

  if (!Hid.SetReportParser(0, &Joy))
    ErrorMessage<uint8_t>(PSTR("SetReportParser"), 1  );

  Joystick.ch1[0] = 'J';
  Joystick.ch2[0] = 'S';
  Joystick.ch3[0] = 'L';
  Joystick.ch1[1] = 'J';
  Joystick.ch2[1] = 'S';
  Joystick.ch3[1] = 'L';
}

void loop()
{
  chksend = 0;
  Usb.Task();
  if (!chksend)
  {
    chksend = 1;
    Joystick.serialn++;
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

    /////////////////////////////////////////////////////////
    // Toggling the Communication Checker Boolean

    if (Joystick.chkupd == 0)
      Joystick.chkupd = 1;
    else
      Joystick.chkupd = 0;

    /////////////////////////////////////////////////////////

    JSout.sendData();
  }
}


JoystickReportParser::JoystickReportParser(JoystickEvents *evt) :
  joyEvents(evt)
{}

void JoystickReportParser::Parse(HID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf)
{
  bool match = true;

  // Checking if there are changes in report since the method was last called
  for (uint8_t i = 0; i < RPT_GAMEPAD_LEN; i++) {
    if ( buf[i] != oldPad[i] ) {
      match = false;
      break;
    }
  }
  // Calling Game Pad event handler
  if (!match && joyEvents) {
    joyEvents->OnGamePadChanged((const GamePadEventData*)buf);

    for (uint8_t i = 0; i < RPT_GAMEPAD_LEN; i++) oldPad[i] = buf[i];
  }
}

void JoystickEvents::OnGamePadChanged(const GamePadEventData *evt)
{
  int x0          =   evt->x;
  int y0          =   evt->y;
  int twist0      =   evt->twist;
  Joystick.slider =   255 - evt->slider;
  Joystick.hat    =   evt->hat;
  int buttons_a0  =   evt->buttons_a;
  int buttons_b0  =   evt->buttons_b;
  int button[13];

  Joystick.forward = 0;
  Joystick.backward = 0;
  Joystick.left = 0;
  Joystick.right = 0;
  Joystick.cw = 0;
  Joystick.ccw = 0;

  if (y0 >= 400 && y0 <= 624) //neutral Y //DEAD ZONES
  {
    Joystick.forward = 0;
    Joystick.backward = 0;
  }
  else if (y0 >= 0 && y0 < 400) //forward
  {
    Joystick.forward = map(y0, 0, 400, 255, 0);
    Joystick.backward = 0;
  }
  else if (y0 > 624 && y0 <= 1023) //reverse
  {
    Joystick.forward = 0;
    Joystick.backward = map(y0, 624, 1023, 0, 255);
  }

  if (x0 >= 300 && x0 <= 724) //neutral X
  {
    Joystick.left = 0;
    Joystick.right = 0;
  }
  else if (x0 >= 0 && x0 < 300) //left
  {
    Joystick.left = map(x0, 0, 300, 255, 0);
    Joystick.right = 0;
  }
  else if (x0 > 724 && x0 <= 1023) //right
  {
    Joystick.left = 0;
    Joystick.right = map(x0, 724, 1023, 0, 255);
  }

  if (twist0 >= 80 && twist0 <= 175) // Z Neutral
  {
    Joystick.cw = 0;
    Joystick.ccw = 0;
  }
  else if (twist0 >= 0 && twist0 < 80)
  {
    Joystick.cw = 0;
    Joystick.ccw = map(twist0, 0, 80, 255, 0);
  }
  else if (twist0 > 175 && twist0 <= 255)
  {
    Joystick.cw = map(twist0, 175, 255, 0, 255);
    Joystick.ccw = 0;
  }

  for (int i = 1; i <= 8; i++)
  {
    Joystick.button[i] = buttons_a0 % 2;
    buttons_a0 = buttons_a0 / 2;
  }
  for (int i = 9; i <= 12; i++)
  {
    Joystick.button[i] = buttons_b0 % 2;
    buttons_b0 = buttons_b0 / 2;
  }

  if (!chksend)
  {
    chksend = 1;
    Joystick.serialn++;

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

    /////////////////////////////////////////////////////////
    // Toggling the Communication Checker Boolean

    if (Joystick.chkupd == 0)
      Joystick.chkupd = 1;
    else
      Joystick.chkupd = 0;

    /////////////////////////////////////////////////////////

    JSout.sendData();
  }
}

#endif // __HIDJOYSTICKRPTPARSER_H__
