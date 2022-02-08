int motor1pin1 = 8;
int motor1pin2 = 7;
int motor1en=9;

int motor2pin1 = 6;
int motor2pin2 = 5;
int motor2en=3;


const byte encoder1PinA = 12;
const byte encoder1PinB = 13;

const byte encoder2PinA = 2;
const byte encoder2PinB = 4;

const int trigPinMid = A5;
const int echoPinMid = A4;

const int trigPinRight = A0;
const int echoPinRight = A1;

const int trigPinLeft = A2;
const int echoPinLeft = A3;


int distanceMid;
int distanceLeft;
int distanceRight;
 long duration , distance ;
const int threshhold = 25;

int cnt = 0;
void setup() {
  // put your setup code here, to run once:
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(motor1en,OUTPUT);
  pinMode(motor2en,OUTPUT);
  
 


  //ultrasonic setup
pinMode(trigPinMid, OUTPUT);
pinMode(echoPinMid, INPUT);
pinMode(trigPinLeft, OUTPUT);
pinMode(echoPinLeft, INPUT);
pinMode(trigPinRight, OUTPUT);
pinMode(echoPinRight, INPUT);
 digitalWrite(motor1pin2, LOW);
    digitalWrite(motor1pin1, HIGH);
        analogWrite(motor1en,70);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW); 
    analogWrite(motor2en ,77);

Serial.begin(115200);



  
}

void loop() {
  // put your main code here, to run repeatedly:   
    Sonar(trigPinMid, echoPinMid);
    distanceMid=distance;
      Sonar(trigPinLeft, echoPinLeft);
    distanceLeft=distance;
      Sonar(trigPinRight, echoPinRight);
    distanceRight=distance;
    Serial.print("Distance reading from Mid in : ");
    Serial.println(distanceMid);
    Serial.print("Distance reading from Right in : ");
    Serial.println(distanceRight);
    Serial.print("Distance reading from Left in : ");
    Serial.println(distanceLeft);

    if(distanceMid < threshhold ) {
    turnLeft();

    
    }
     
   
   Serial.println(digitalRead(encoder1PinA)-digitalRead(encoder2PinA));

   
    delay(1000);
    





}
void Sonar(int trig , int echo){
digitalWrite(trig, LOW);
delayMicroseconds(2);
digitalWrite(trig, HIGH);
delayMicroseconds(10);
digitalWrite(trig, LOW);
duration = pulseIn(echo, HIGH);
distance = (duration/2) / 29.1;

}

void turnLeft(){
  cnt = 0;
  
  
  digitalWrite(motor1en, LOW);
  digitalWrite(motor2en, LOW);
  delay(2000);
  while(cnt != 13){
    digitalWrite(motor1en,100);
    cnt++;
    delay(50);
  }
  digitalWrite(motor1pin1, HIGH);
  analogWrite(motor1en,70);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW); 
    analogWrite(motor2en ,77); 
  
  }
