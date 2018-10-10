/* 
 Final Code of MSE 2202 Project
 Group 6
 Karam Albakri, Alan Harris, Peter McLay, Ariel Tebeka
*/

#include <Servo.h>

/**************** VARIABLE DECLARATION *************/

//Serial Print
//#define DEBUG_MOTORS
//#define DEBUG //front sensor
//#define DEBUG2 //arm sensor

// Wall Tracking Ultrasonic Sensors and Variables
int fore[2] = {A5, A4}; //first is trig, second is echo
int aft[2] = {6, 5}; //first is trig, second is echo
int bow[2] = {A3, A2};
int trig = 0, echo = 1;

//IR Sensors
const int led = 13;
const int led2 = 10;

//Arm
int Echo2 = A0;
int Trig2 = A1;
int flag = 0;
Servo myservo;
long cubeFinder = 0;
int cubeFlag;
 
//Lift Motors and Variables
Servo liftMotor;
Servo cubeMotor;
Servo forkLeft;
Servo forkRight;
long interval = 0;
bool forkFlag = true;
bool backUpFlag = true;
long backUpTimer;

//Drive Motors
const int ci_rightMotor = 8;
const int ci_leftMotor = 9;
unsigned int leftMotorSpeed;
unsigned int rightMotorSpeed;
Servo rightMotor;
Servo leftMotor;

//Cases
unsigned int state = 1;
unsigned int stage = 0;

// Wall Detection Variables
int wallFlag = 0;
long turnTimer = 0;
bool turnFlag = true;
bool pingQueue = true;
long foreDuration = 0, aftDuration = 0, bowDuration = 0, direct = 0, pingTimer = 0, armDuration = 0;
int armFlag = 0;

// Front Limitswitch
const int limitswitch = 2;    
int limitswitchstate = 1;         
const int limitswitch2 = 11;
int limitswitchstate2 = 1;

//Timer Variables
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 2000;
const unsigned long period_spin = 3000;
unsigned long _startMillis;

/*********************** Set Up ************************/
void setup() {
  //set pin modes for the fore ultrasonic
  pinMode(fore[0], OUTPUT);
  pinMode(fore[1], INPUT);
  //set pin modes for the aft ultrasonic
  pinMode(aft[0], OUTPUT);
  pinMode(aft[1], INPUT);
  //set pin modes for the bow ultrasonic
  pinMode(bow[0], OUTPUT);
  pinMode(bow[1], INPUT);

  //IR
  pinMode(led, INPUT);
  pinMode(led2, OUTPUT);
    
  //Arm
  pinMode(Echo2, INPUT);
  pinMode(Trig2, OUTPUT);
  myservo.attach(10);  

  // Cube Detatchment Motor
  cubeMotor.attach(7);
  
  //Fork Lift Motors
  forkLeft.attach(3); 
  forkRight.attach(4); 
  
  //Motors
  pinMode(ci_rightMotor, OUTPUT);
  rightMotor.attach(ci_rightMotor);
  pinMode(ci_leftMotor, OUTPUT);
  leftMotor.attach(ci_leftMotor);

  //Limit Switch 1 and 2
  pinMode(limitswitch, INPUT);
  digitalWrite(limitswitch, HIGH);
  pinMode(limitswitch2, INPUT);
  digitalWrite(limitswitch2, HIGH);
  
  // initialize serial communication:
  Serial.begin(9600);
}

/*************** Ping Function *****************/
int ping(double distance = 4)
{
 double analogDist = distance*58;
 if(pingQueue){
  pingTimer = millis();
  digitalWrite(fore[trig], LOW);
  delayMicroseconds(2);
  digitalWrite(fore[trig], HIGH);
  delayMicroseconds(5);
  digitalWrite(fore[trig], LOW);
  digitalWrite(fore[echo], HIGH);
  foreDuration = pulseIn(fore[echo], HIGH);
  pingQueue = false;
 } 
 if (millis() >= (pingTimer + 50) && !pingQueue){
  digitalWrite(aft[trig], LOW);
  delayMicroseconds(2);
  digitalWrite(aft[trig], HIGH);
  delayMicroseconds(5);
  digitalWrite(aft[trig], LOW);
  digitalWrite(aft[echo], HIGH);
  aftDuration = pulseIn(aft[echo], HIGH);
  pingQueue = true;
 }  
 if (aftDuration > 0 && aftDuration < 10000 && foreDuration > 0 && foreDuration < 10000){
  if (stage == 1){
   if (foreDuration - aftDuration < 20 && aftDuration < 280){
    return 4;
   }
  }
  if (foreDuration > (aftDuration + 40)) return 2; //Straighten out to the right
  if (aftDuration > (foreDuration + 40)) return 3; //Straighten out to the left
  if (foreDuration > (analogDist + 15) && aftDuration > (analogDist + 15)) return 2; //To far away, drive right
  if (foreDuration < (analogDist - 15) && aftDuration < (analogDist - 15)) return 3; //To close, drive left
  if (foreDuration > (aftDuration + 10)) return 2; //Drive Right
  else if (foreDuration < (aftDuration - 10)) return 3; //Drive Left
  else return 1; //Drive Straight 
 }
}

/*************** PingFront *******************/
long pingFront(){
  digitalWrite(bow[trig], LOW);
  delayMicroseconds(2);
  digitalWrite(bow[trig], HIGH);
  delayMicroseconds(5);
  digitalWrite(bow[trig], LOW);
  digitalWrite(bow[echo], HIGH);
  bowDuration = pulseIn(bow[echo], HIGH);
  Serial.println(bowDuration/58.00);
  return bowDuration/58.00;  
}

/*********************ping Arm *********************/
bool pingArm(){
  digitalWrite(Trig2,LOW);
  delayMicroseconds(2);
  digitalWrite(Trig2, HIGH);
  delayMicroseconds(5);
  digitalWrite(Trig2, LOW);
  digitalWrite (Echo2, HIGH);
  armDuration = pulseIn(Echo2, HIGH);
  Serial.println(armDuration/58.00);
  if (armDuration/58.00 < 15){
   armFlag++;
   if (armFlag > 10){
    armFlag = 0;
    return true;    
   }  
  }
  else{  
   return false;
  }
}
  
/********************** HitWall ****************/
bool hitWall(){
  long dist = pingFront();
  if(dist <= 9.00){
   wallFlag++;
  }
  else{
   wallFlag = 0;
  }
  if (wallFlag == 5){
   return true;
  }
  else{
   return false;
  }
}
/*********************** MAIN CODE **********************/
void loop(){  

switch(state){
case 0:{
   leftMotorSpeed = 1500;
   rightMotorSpeed = 1500;
   pingArm(); 
   state = 1;
   break; 
}

case 1:{
switch(stage){

/***************** CASE 0 ***********************/
case 0:{    
   leftMotorSpeed = 1630;
   rightMotorSpeed = 1630;
   forkLeft.write(115);
   forkRight.write(25); 
   if (hitWall()){
    leftMotorSpeed = 1500;
    rightMotorSpeed = 1500;
    myservo.write(180);
    cubeMotor.write(37);    
    stage = 1;
    break;
   }
   break; 
}

/********************** CASE 1 *******************/
case 1:{
   Serial.print("Stage: ");
   Serial.println(stage);
   leftMotorSpeed = 1370;
   rightMotorSpeed = 1630;
   direct = ping();
   if (direct == 4){
    leftMotorSpeed = 1500;
    rightMotorSpeed = 1500;
    stage = 2;
    break;
   }     
   break;      
}

/******************** CASE 2 ************************/
case 2:{
   direct = ping();
   myservo.write(180);
   Serial.print("Stage: ");
   Serial.println(stage);
   if (pingArm()){
    leftMotorSpeed = 1500;
    rightMotorSpeed = 1500;
    stage = 3;
    break;
   }
   if (direct == 3){
    leftMotorSpeed = 1620;
    rightMotorSpeed = 1660;
   }
   if (direct == 2){
    leftMotorSpeed = 1660;
    rightMotorSpeed = 1620;
   }
   if (direct == 1){
    leftMotorSpeed = 1630;
    rightMotorSpeed = 1630;
   }
   if (pingFront() < 9){
    if (turnFlag){
     turnTimer = millis();
     turnFlag = false;
    }
    if (millis() > turnTimer + 2000){
     stage = 1;
    }
    else{
     rightMotorSpeed = 1630;
     leftMotorSpeed = 1370;
    }
   }
   break;
}

/*************************** CASE 3 ******************************/
case 3: {
   Serial.print("stage: ");
   Serial.println(stage);
   long timer = millis();
   if (backUpFlag){
    leftMotor.writeMicroseconds(1500);
    rightMotor.writeMicroseconds(1500);
    backUpTimer = millis();
    backUpFlag = false;
    delay(300);
   }
   if (timer - backUpTimer < 600){
    leftMotorSpeed = 1400;
    rightMotorSpeed = 1400;
   }
   else{
    leftMotor.writeMicroseconds(1500);
    rightMotor.writeMicroseconds(1500);
    delay(1000);
    myservo.write(9);
    delay(1500);
    limitswitchstate2 = digitalRead(limitswitch2);
    delay(500);
    if (limitswitchstate2 == LOW){
     stage = 4;
     digitalWrite(led,LOW);
     startMillis = millis();
     break;
    }
    leftMotor.writeMicroseconds(1620);
    rightMotor.writeMicroseconds(1600);
    myservo.write(180);
    while(cubeFinder < 150){
     Serial.println(cubeFinder);
     cubeFinder++;
    }
   cubeFinder = 0;
   stage = 2;
   armFlag = 0;
   }
   break;
}

/************************* CASE 4 *******************************/
case 4: {
  Serial.print("Stage: ");
  Serial.println(stage);
  if (digitalRead(led)==HIGH){
   rightMotorSpeed = 1630;
   leftMotorSpeed = 1630;
   stage = 6;
   break;
  }
  else{
   currentMillis = millis();
   if (currentMillis - startMillis >= period_spin){
    startMillis = currentMillis;
    stage = 5;
    break;
   }
   else if (Distance_test() < 17){
    rightMotorSpeed = 1380;
    leftMotorSpeed = 1380;
   }
   else{
    rightMotorSpeed = 1640;
    leftMotorSpeed = 1500;
   }
  }
  break;
}

/************************* CASE 5 *******************/
case 5: {
  Serial.print("Stage: ");
  Serial.println(stage);
  currentMillis = millis();
  if (currentMillis - startMillis >= period){
   startMillis = currentMillis;
   stage = 4;
   break; 
  }
  if (Distance_test() < 17){
   rightMotorSpeed = 1380;
   leftMotorSpeed = 1380;
   break;
  }
  else if (Distance_test_side() < 12 && Distance_test()  <  12 ){
   rightMotorSpeed = 1380;
   leftMotorSpeed = 1380;
   break;
  }
  else{
   rightMotorSpeed = 1700;
   leftMotorSpeed = 1700;
  }
  break;
}

/************************* CASE 6 *******************/
case 6: {
  Serial.print("Stage: ");
  Serial.println(stage);
  digitalWrite(led, HIGH);
  limitswitchstate = digitalRead(limitswitch);
  if (limitswitchstate == LOW){
    rightMotorSpeed = 1500;
    leftMotorSpeed = 1500;
    Serial.println("Switch hit");
    stage = 7;
    break;
  }
  currentMillis = millis();
  if (currentMillis - _startMillis >= 1000){
    _startMillis = currentMillis;
    digitalWrite(led, LOW);
    Serial.print("Case 8");
    rightMotorSpeed = 1500;
    leftMotorSpeed = 1500;
    stage = 8;
    break;
  }
    break;
  }

/***************** CASE 8 **************/
case 8:
{
 long _currentMillis = millis();
 if (_currentMillis - _startMillis <= 3000){
  rightMotorSpeed = 1500;
  leftMotorSpeed = 1500;
  Serial.print("Time elapsed");
 } 
 else{
  rightMotorSpeed = 1500;
  leftMotorSpeed = 1630;
 }
  
 if (digitalRead(led)== HIGH){
  startMillis = currentMillis;
  rightMotorSpeed = 1630;
  leftMotorSpeed = 1630;
  stage = 6;
  break;
 }
 break;
}

/********************** Case 7 *****************/
case 7: { //Picking the Pyramid
  Serial.print("Stage: ");
  Serial.println(stage);
  long timer = millis();
  if (forkFlag){
    interval = millis();
    forkFlag = false;
    } 
  if(timer - interval < 600){ //back up
   leftMotorSpeed = 1400;
   rightMotorSpeed = 1400;
   Serial.println("1");
  }
  if (timer - interval > 600 && timer - interval < 2000){ //stop
   leftMotorSpeed = 1500;
   rightMotorSpeed = 1500;
   Serial.println("2");
  }
  if(timer - interval > 2000 && timer - interval < 3750){// drop the lift
   forkLeft.write(15);
   forkRight.write(125);
   Serial.println("3");
  }
  if(timer - interval > 3750 && timer - interval < 5000){ // chill
  liftMotor.write(1500);
  Serial.println("4");
  }
  if(timer - interval > 5500 && timer - interval < 16000){ //go forward
    leftMotorSpeed = 1600;
    rightMotorSpeed = 1600;
    myservo.write(180);
    Serial.println("1");
  }
  if (timer - interval > 16000 && timer - interval < 16800){ //lift
    forkLeft.write(100);
    forkRight.write(40);  
  }
  if (timer - interval > 16800 && timer - interval < 18000){ //stop
    leftMotorSpeed = 1500;
    rightMotorSpeed = 1500;
    Serial.println("2");
  } 
  if(timer - interval > 18500 && timer - interval < 30000){ //detach cube
    cubeMotor.write(120);
    myservo.write(180);
    delay(1000);
    myservo.write(7);
    delay(1000);
    cubeMotor.write(30);
    delay(2000);
    myservo.write(180);
    delay(2000);
    cubeMotor.write(120);
  }
  if (timer - interval > 30000 && timer - interval < 32000){ //drop the lift
   forkLeft.write(30);
   forkRight.write(115);
   Serial.println("6");
  }
  if(timer - interval > 32000 && timer - interval < 33000){ // chill
   liftMotor.write(1500);
   Serial.println("7");
  }
  if (timer - interval > 33000 && timer - interval < 33500){ // back up
   leftMotorSpeed = 1400;
   rightMotorSpeed = 1400;
   Serial.println("8");
  }
  if (timer - interval > 33500 && timer - interval < 34500){ //stop
   leftMotorSpeed = 1500;
   rightMotorSpeed = 1500;
   Serial.println("9");
  }
  if (timer - interval > 34500 && timer - interval < 35750){ // pick up lift
   forkLeft.write(110);
   forkRight.write(30);
   Serial.println("8");
  }
  break;
}
    
} 
}
}

/******************* END OF CASES *************************/

leftMotor.writeMicroseconds(leftMotorSpeed);
rightMotor.writeMicroseconds(rightMotorSpeed);    
#ifdef DEBUG_MOTORS
Serial.print(", Left = ");
Serial.print(leftMotorSpeed);
Serial.print(", Right = ");
Serial.println(rightMotorSpeed);
#endif 
}

/******************* END OF LOOP *******************/

/******** DISTANCE TEST FRONT SENSOR ***************/
int Distance_test()   
{
  digitalWrite(bow[0], LOW);   
  delayMicroseconds(2);
  digitalWrite(bow[0], HIGH);  
  delayMicroseconds(5);
  digitalWrite(bow[0], LOW);   
  float Fdistance = pulseIn(bow[1], HIGH);  
  Fdistance= Fdistance/58;
  #ifdef DEBUG
   //Serial.print("Distance1=");
   //Serial.println(Fdistance);
  #endif       
  return (int)Fdistance;
}  

/***************DISTANCE FOR ARM ******************/
int Distance_test2(){
  digitalWrite(Trig2, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig2, HIGH);  
  delayMicroseconds(5);
  digitalWrite(Trig2, LOW);   
  float Fdistance = pulseIn(Echo2, HIGH);  
  Fdistance= Fdistance/58;  
  #ifdef DEBUG2
   //Serial.print("Distance2=");
   //Serial.println(Fdistance);
  #endif     
  return (int)Fdistance;
}
/************ DISTANCE FOR SIDE SENSOR ***********/
int Distance_test_side(){
  digitalWrite(aft[0], LOW);   
  delayMicroseconds(2);
  digitalWrite(aft[0], HIGH);  
  delayMicroseconds(5);
  digitalWrite(aft[0], LOW);   
  float Fdistance = pulseIn(aft[1], HIGH);  
  Fdistance= Fdistance/58;
  #ifdef DEBUG
    //Serial.print("Distance1=");
    //Serial.println(Fdistance);
  #endif       
  return (int)Fdistance;
}
