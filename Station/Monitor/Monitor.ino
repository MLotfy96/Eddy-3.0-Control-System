#include <TVout.h>
#include <EasyTransfer.h>
#include <fontALL.h>

EasyTransfer MNin;
TVout TV;

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

bool x;
char bt[30];
int i = 0;
unsigned long long start = 0;

void setup()  {
  Serial.begin(230400);
  MNin.begin(details(Feedback), &Serial);
  TV.begin(_NTSC, 120, 95); //125 100
  TV.select_font(font6x8);
  start = millis();
}

void loop()
{
  MNin.receiveData();

  if (Feedback.ch1[0] != 'F' || Feedback.ch2[0] != 'B' || Feedback.ch3[0] != 'K' || Feedback.ch1[0] != 'F' || Feedback.ch2[0] != 'B' || Feedback.ch3[0] != 'K')
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

  TV.println(0, 0, "Invictus ROV Co.");
  TV.println();

  TV.print("H ");
  TV.print(Feedback.heading);
  TV.print(" ");
  if ((Feedback.heading > 338 ) || (Feedback.heading >= 0  && Feedback.heading <=  23)) TV.println("N   ");
  else if (Feedback.heading >  23 && Feedback.heading <=  68) TV.println("NE  ");
  else if (Feedback.heading >  68 && Feedback.heading <= 113) TV.println("E   ");
  else if (Feedback.heading > 113 && Feedback.heading <= 158) TV.println("SE  ");
  else if (Feedback.heading > 158 && Feedback.heading <= 203) TV.println("S   ");
  else if (Feedback.heading > 203 && Feedback.heading <= 248) TV.println("SW  ");
  else if (Feedback.heading > 248 && Feedback.heading <= 293) TV.println("W   ");
  else if (Feedback.heading > 293 && Feedback.heading <= 338) TV.println("NW  ");
  //TV.print(0,24,"   ");
  TV.print("P ");
  TV.print(Feedback.pitch);
  TV.print("  ");
  TV.print("R ");
  TV.print(Feedback.roll);
  TV.print("  ");

  TV.print(0, 40, " Internal External");
  TV.print(0, 48, "P ");
  TV.print(Feedback.ip);
  TV.println(60, 48, Feedback.ep);
  TV.print("T ");
  TV.print(Feedback.it);
  TV.println(60, 56, Feedback.et);

  /*
  TV.print("EP\t");
  TV.print(Feedback.ep);
  TV.print(" kPa\t");
  TV.print("ET ");
  TV.print(Feedback.et);
  TV.println(" C");
  */

  bt[i] = Feedback.bt;
  TV.println("BT");
  //TV.select_font(font4x6);
  TV.println(bt);
  //TV.select_font(font6x8);
  /*
   if(x == Feedback.update)
   if((millis() - start)%100 && x == Feedback.update){
     memset(&Feedback,0,sizeof(FEEDBACK));
     Feedback.btchar='x';
   }

   x = Feedback.update;
   */
  i++;
  if (i >= 28)i = 0;
}
