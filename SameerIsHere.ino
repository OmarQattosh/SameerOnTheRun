#include <PID_v1.h>
int motor1pin1 = 7;
int motor1pin2 = 8;
int motor1en = 5;

int motor2pin1 = 9;
int motor2pin2 = 10;
int motor2en = 6;
int diskslot = 12;
const byte encoder1PinA = 3;
const byte encoder1PinB = 13;

const byte encoder2PinA = 2;
const byte encoder2PinB = 12;

const int trigPinMid = A5;
const int echoPinMid = A4;

const int trigPinRight = A2;
const int echoPinRight = A3;

const int trigPinLeft = A0;
const int echoPinLeft = A1;
unsigned int timer1_counter;
int distanceMid;
int distanceLeft;
int distanceRight;
long duration , distance ;
const int threshhold = 10; 
unsigned int Pulse_L = 0;
unsigned int Pulse_R = 0;
int cnt = 0;
volatile int cntL =0;
volatile int cntM =0;

double Left_rpm = 0.0;
double Right_rpm = 0.0;
double pwmR = 0.0;
double pwmL = 0.0;
double rpm_set_Left = 0.0;
double rpm_set_Right = 0.0;
float diskslots = 12 ;
PID pidL(&Left_rpm, &pwmL, &rpm_set_Left, 2, 5, 1, DIRECT);
PID pidR(&Right_rpm, &pwmR, &rpm_set_Right, 2, 5, 1, DIRECT);


void ISR_countL()
{
  Pulse_L++; // increment Left Motor counter value
  // Serial.println("inside of interrupt");
}

// Right Motor pulse count ISR
void ISR_countR()
{
  Pulse_R++; // increment Right Motor counter value
}
ISR(TIMER1_OVF_vect)
{


}
void setup() {
  // put your setup code here, to run once:
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(motor1en, OUTPUT);
  pinMode(motor2en, OUTPUT);
  pinMode(encoder2PinA, INPUT);
  pinMode(encoder1PinA, INPUT);
  pinMode(trigPinMid, OUTPUT);
  pinMode(echoPinMid, INPUT);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor1pin1, HIGH);
  analogWrite(motor1en, 60);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
  analogWrite(motor2en , 60);
  pidL.SetMode(AUTOMATIC);
  pidR.SetMode(AUTOMATIC);
  /*----------- timer setup ---------------*/
  noInterrupts(); // disable all interrupts
  TCCR1A         = 0;
  TCCR1B         = 0;
  timer1_counter = 31250; // preload timer 65536-16MHz/256/2Hz (34286 for
  // 0.5sec) (31250 for 0.5sec)

  TCNT1 = timer1_counter; // preload timer
  TCCR1B |= (1 << CS12);  // 256 prescaler
  TIMSK1 |= (1 << TOIE1); // enable timer overflow interrupt
  interrupts();   // enable all interrupts
  /*----------- timer setup ---------------*/

  attachInterrupt(digitalPinToInterrupt(encoder2PinA), ISR_countL, RISING); // Increase counter 1 when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), ISR_countR, RISING);
  //ultrasonic setup


  Serial.begin(115200);




}

void loop() {
  // put your main code here, to run repeatedly:
  //    Sonar(trigPinMid, echoPinMid);
  //    distanceMid=distance;
  //      Sonar(trigPinLeft, echoPinLeft);
  //    distanceLeft=distance;
  //      Sonar(trigPinRight, echoPinRight);
  //    distanceRight=distance;
  // Serial.print("Distance reading from Mid in : ");
  // Serial.println(distanceMid);
  on();
  
  if (!wallLeft()) {
    turnLeft();
  }
  while (wallfront()) {
      turnRight();
  }
  on();

}



bool wallLeft() {
  Sonar(trigPinLeft, echoPinLeft);
  distanceLeft = distance;
  Serial.print("distance Left: ");
  Serial.print(distanceLeft);
  Serial.println();
  
  if (distanceLeft < threshhold && cntL==3 ) {
    cntL++;
    return true;
  } else return false;
}

bool wallfront() {
  Sonar(trigPinMid, echoPinMid);
  distanceMid = distance;
  Serial.print("distance Mid is :");
  Serial.print(distanceMid);
  Serial.println();
  
  if (distanceMid < threshhold ) {
    cntM++;
    return true;
  } else return false;
}

void Sonar(int trig , int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  distance = (duration / 2) / 29.1;

}
void on() {
//  if (pidL.Compute()) {
//    pidL.Compute();
//    analogWrite(motor1en, pwmL);
//  }
  
    analogWrite(motor1en, 60);
 
//  if (pidR.Compute()) {
//    pidR.Compute();
//    analogWrite(motor2en , pwmR);
//  }
  
    analogWrite(motor2en, 60);
  
}
void turnLeft() {
  cnt = 0;
  Pulse_L = 0;
  Pulse_R = 0;


  digitalWrite(motor1en, LOW);
  digitalWrite(motor2en, LOW);
  delay(1000);
  digitalWrite(motor1en, HIGH);
  while (cnt < 165 ) {
    //   Serial.println(Pulse_R);
    cnt++;
    delay(1);

  }
  cntL= 0;

  analogWrite(motor2en , 60);
  analogWrite(motor1en , 60);
  //stopp();

}
void turnRight() {
  cnt = 0;
  
  Pulse_L = 0;
  Pulse_R = 0;
  Serial.print("distance Left: ");
  Serial.print(distanceLeft);

  digitalWrite(motor1en, LOW);
  digitalWrite(motor2en, LOW);
  delay(1000);
  digitalWrite(motor2en, HIGH);
  while (cnt < 165 ) {
    //   Serial.println(Pulse_R);
    cnt++;
    delay(1);

  }

  analogWrite(motor2en , 60);
  analogWrite(motor1en , 60);
  //stopp();

}




void stopp() {
  digitalWrite(motor1en, LOW);
  digitalWrite(motor2en, LOW);
  delay(5000);
}
