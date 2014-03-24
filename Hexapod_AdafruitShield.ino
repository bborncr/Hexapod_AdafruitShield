                                     // IRremote library written by Ken Shirriff
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <IRLib.h> 
#define MY_PROTOCOL SONY
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want

float SERVOFREQ = 50;
float pulseconstant;                              

int angle;                                                 // determines the direction/angle (0°-360°) that the robot will walk in 
int rotate;                                                // rotate mode: -1 = anticlockwise, +1 = clockwise, 0 = no rotation
int Speed;                                                 // walking speed: -15 to +15 
int Stride;                                                // size of step: exceeding 400 may cause the legs to hit each other

int RECV_PIN = 11;                                  
IRrecv irrecv(RECV_PIN);
IRdecode My_Decoder;

#define ONE 0xb92
#define TWO 0x80b92
#define THREE 0x40b92
#define FOUR 0xc0b92
#define FIVE 0x20b92
#define SIX 0xa0b92
#define SEVEN 0x60b92
#define EIGHT 0xe0b92
#define NINE 0x10b92
#define PLAY 0x4cb92
#define STOP 0x1cb92

void setup()
{
  
  Serial.begin(9600);                        // IR test mode displays IR receiver values on serial monitor
  irrecv.enableIRIn();      // Start the receiver
  pulseconstant = (1000000/SERVOFREQ)/4096;

  pwm.begin();
  pwm.setPWMFreq(SERVOFREQ);
  
  for(int i=0;i<12;i++)
  {
    servoWrite(i,1500);                       // initialize servos to center
  }
  
}

void loop()
{ 
  if (irrecv.GetResults(&My_Decoder))                             // check for IR command
  {                                                        // change IRC comparison values to suit your TV, DVD, Stereo remote
    My_Decoder.decode();
    //My_Decoder.DumpResults();
    unsigned long IRC=My_Decoder.value;
    Serial.println(IRC,HEX);                     // display value from IR receiver on serial monitor in test mode

    if(IRC==FIVE)                              // STOP
    {
      Speed=0;
      angle=0;
      rotate=0;
    }

    if(IRC==TWO)                              // FORWARD
    {
      Speed=10;
      rotate=0;
      angle=0;
    }

    if(IRC==EIGHT)                             // REVERSE    
    {
      Speed=-10;
      rotate=0;
      angle=0;
    }

    if(IRC==STOP)                                          // ROTATE CLOCKWISE  
    {
      Speed=10;
      rotate=1;
      angle=0;
    }

    if(IRC==PLAY)                                          // ROTATE COUNTER CLOCKWISE  
    {
      Speed=10;
      rotate=-1;
      angle=0;
    }

    if(IRC==ONE)                                            // 45 DEGREES    
    {
      Speed=10;
      rotate=0;
      angle=45;
    }

    if(IRC==FOUR)                                          // 90 DEGREES    
    {
      Speed=10;
      rotate=0;
      angle=90;
    }

    if(IRC==THREE)                                          // 135 DEGREES    
    {
      Speed=10;
      rotate=0;
      angle=135;
    }

    if(IRC==NINE)                                           // 225 DEGREES    
    {
      Speed=10;
      rotate=0;
      angle=225;
    }

    if(IRC==SIX)                                          // 270 DEGREES    
    {
      Speed=10;
      rotate=0;
      angle=270;
    }

    if(IRC==SEVEN)                                          // 315 DEGREES    
    {
      Speed=10;
      rotate=0;
      angle=315;
    }
    irrecv.resume();                                       // receive the next value
  }


  if (angle<0) angle+=360;                                 // keep travel angle within 0°-360°
  if (angle>359) angle-=360;                               // keep travel angle within 0°-360°
  Walk();                                                  // move legs to generate walking gait
  delay(15);
}


void Walk()                                                // all legs move in a circular motion
{
  if(Speed==0)                                             // return all legs to default position when stopped
  {
    Stride-=25;                                            // as Stride aproaches 0, all servos return to center position
    if(Stride<0) Stride=0;                                 // do not allow negative values, this would reverse movements
  }
  else                                                     // this only affects the robot if it was stopped
  {
    Stride+=25;                                            // slowly increase Stride value so that servos start up smoothly
    if(Stride>450) Stride=450;                             // maximum value reached, prevents legs from colliding.
  }

  float A;                                                 // temporary value for angle calculations
  double Xa,Knee,Hip;                                      // results of trigometric functions
  static int Step;                                         // position of legs in circular motion from 0° to 360°                               

  for(int i=0;i<6;i+=2)                                    // calculate positions for odd numbered legs 1,3,5
  {
    A=float(60*i+angle);                                   // angle of leg on the body + angle of travel
    if(A>359) A-=360;                                      // keep value within 0°-360°

    A=A*PI/180;                                            // convert degrees to radians for SIN function

    Xa=Stride*rotate;                                      // Xa value for rotation
    if(rotate==0)                                          // hip movement affected by walking angle
    {
      Xa=sin(A)*-Stride;                                   // Xa hip position multiplier for walking at an angle
    }

    A=float(Step);                                         // angle of leg
    A=A*PI/180;                                            // convert degrees to radians for SIN function
    Knee=sin(A)*Stride;
    Hip=cos(A)*Xa;

    servoWrite(i*2,1500+int(Knee));
    servoWrite(i*2+1,1500+int(Hip));
    //sv[i*2].writeMicroseconds(svc[i*2]+int(Knee));         // update knee  servos 1,3,5
    //sv[i*2+1].writeMicroseconds(svc[i*2+1]+int(Hip));      // update hip servos 1,3,5
  }

  for(int i=1;i<6;i+=2)                                    // calculate positions for even numbered legs 2,4,6
  {
    A=float(60*i+angle);                                   // angle of leg on the body + angle of travel
    if(A>359) A-=360;                                      // keep value within 0°-360°

    A=A*PI/180;                                            // convert degrees to radians for SIN function
    Xa=Stride*rotate;                                      // Xa value for rotation
    if(rotate==0)                                          // hip movement affected by walking angle
    {
      Xa=sin(A)*-Stride;                                   // Xa hip position multiplier for walking at an angle
    }

    A=float(Step+180);                                     // angle of leg
    if(A>359) A-=360;                                      // keep value within 0°-360°
    A=A*PI/180;                                            // convert degrees to radians for SIN function
    Knee=sin(A)*Stride;
    Hip=cos(A)*Xa;
    
    servoWrite(i*2,1500+int(Knee));
    servoWrite(i*2+1,1500+int(Hip));

    //sv[i*2].writeMicroseconds(svc[i*2]+int(Knee));         // update knee  servos 2,4,6
    //sv[i*2+1].writeMicroseconds(svc[i*2+1]+int(Hip));      // update hip servos 2,4,6
  }

  Step+=Speed;                                             // cycle through circular motion of gait
  if (Step>359) Step-=360;                                 // keep value within 0°-360°
  if (Step<0) Step+=360;                                   // keep value within 0°-360°
}

// you can use this function if you'd like to set the pulse length in useconds
// e.g. servoWrite(0, 1500) is a 1500usecond pulsewidth
void servoWrite(uint8_t n, float pulse) {
  float pulsetick;
  pulsetick = pulse/pulseconstant;
  //Serial.println(pulsetick);
  pwm.setPWM(n, 0, pulsetick);
}
