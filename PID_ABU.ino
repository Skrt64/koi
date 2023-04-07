#include <PID_v1.h>

// trig
int pingPin = 13;
// echo
int inPin = 12;

#define PIN_OUTPUT 25

#define SOUND_SPEED 0.034
long duration;
float distanceCm;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double last;

//Specify the links and initial tuning parameters
//double Kp=0.8, Ki=1, Kd=0.55;
double Kp=0.6, Ki=0.8, Kd=0.125;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//min = 0
//max = 460

void setup()
{
  Serial.begin(9600);
  //initialize the variables we're linked to

  Input = readUltra();
  Setpoint = 430;


  
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  
  Input = readUltra();

  myPID.Compute();
  Serial.println(String(Setpoint)+" "+String(Input)+" "+String(Output));
  analogWrite(PIN_OUTPUT, Output);
  delay(15);
}

double readUltra(){
  long duration, cm;

  pinMode(pingPin, OUTPUT);


  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  pinMode(inPin, INPUT);
  duration = pulseIn(inPin, HIGH);

  cm = microsecondsToMillimeters(duration);

  return (500-cm);
}

long microsecondsToInches(long   microseconds)
{return microseconds / 74 / 2;}
long microsecondsToCentimeters(long   microseconds)
{return microseconds / 29 / 2;}
long microsecondsToMillimeters(long   microseconds)
{return microseconds / 2.9 / 2;}


