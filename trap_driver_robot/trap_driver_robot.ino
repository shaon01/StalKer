#include <Encoder.h>   
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); //i2c bus adress
Encoder rightEnc(2, 5);  //reading right motor encoder pin RB-->2, RA-->5
Encoder leftEnc(3, 6);   //reading left motor encoder pin  LA-->3,LB-->6

bool received = false;

float v_max = 1.0;  //in mm/ms
float a_max = 0.01; //in mm/ms2
int dt = 10;

float runRobot(int sped){  /// to run the robot call this function
  
  Adafruit_DCMotor *leftWheel  = AFMS.getMotor(1);   //left motor
  Adafruit_DCMotor *rightWheel = AFMS.getMotor(4);   //right motor

  long oldleftEnc = leftEnc.read();     //reading encoders
  long oldrightEnc = rightEnc.read();

  rightWheel -> setSpeed(sped);
  leftWheel -> setSpeed(sped);
  
  leftWheel -> run(FORWARD);
  rightWheel -> run(FORWARD);

  delay(dt);    //20ms is the dt

  long newleftEnc = leftEnc.read();     //reading encoders
  long newrightEnc = rightEnc.read();

  //taking the average of two wheel encoder and then convert to distance in mm ; 1 tick = 0.5mm
  float dist = ((newrightEnc - oldrightEnc) + (newleftEnc - oldleftEnc))/4;

  return dist;
  
  }

void goForward(int distance){  //get distance in cm 
  
  float p_goal = distance*10; //convert distance to mm
  float traveled = 0;
  float v_curr = 0.1; //initial vleocity
  float dp_min = 0;
  float toGo = p_goal;
  float diff ;

   while((p_goal - traveled)>0){

    dp_min = (v_curr*v_curr)/a_max;  //critical distance to strat breaking
    toGo = p_goal - traveled;  // how far to go
 
    if(dp_min<toGo){
      v_curr = min(v_max, v_curr+a_max*dt); //take the minimum of 
      
      // 1.5mm/ms speed for pwm 200, so coverstion is (200/1.5)*v_curr
      int spedd = 133*v_curr;
      diff = runRobot(spedd); // run the robot at calculated speed
      
      }

    else{

      v_curr  = v_curr - a_max*dt;

      int spedd = 133*v_curr;
      diff = runRobot(spedd); // run the robot 

      
      } 

    traveled = traveled + diff;
    
    }
    //stop motor after reaching the position
    stopRobot();
    
  
  }

void stopRobot(){
  
    Adafruit_DCMotor *leftWheel  = AFMS.getMotor(1);
    Adafruit_DCMotor *rightWheel = AFMS.getMotor(4);

    leftWheel -> run(RELEASE);     
    rightWheel -> run(RELEASE);
    
  
  }

void goBackward(sped){
  //this line will just for git.
  }

void setup() {

  Serial.begin(115200);
  Serial.flush();
  
  AFMS.begin();

}

void loop(){

  byte msg[10];  //massage storing variable;
  int count = 0;
  while (Serial.available() > 0) {

    byte inByte = (byte)Serial.read(); //reading serial
    msg[count] = inByte;

    count++;
    received = true;
  }

  if(received){

    if(msg[0] == 'f'){
      goForward(msg[1]);
    
    }

    if (msg[0] == 's'){

      stopRobot();
      Serial.println("its stopped");
  
      
      }
  }

      received =false;
      delay(10);

    
  
  }



 
