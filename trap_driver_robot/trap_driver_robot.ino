#include <Encoder.h>   
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); //i2c bus adress
Encoder rightEnc(2, 5);  //reading right motor encoder pin RB-->2, RA-->5
Encoder leftEnc(3, 6);   //reading left motor encoder pin  LA-->3,LB-->6

bool received = false;

float v_max = 1.0;  //maximum speed of robot in mm/ms   1.5mm/ms->200(pwm)
float a_max = 0.01; //in mm/ms2
int dt = 10;  //sampling time

//////////****************************///////////////
//Basic motor driving function
///////////////**********************///////////


float goForward(int sped){  /// to run the robot forward call this function
  
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
  float dist = (((newrightEnc - oldrightEnc) + (newleftEnc - oldleftEnc))/2)*0.5;
  Serial.print("old left encoder :");Serial.print(oldleftEnc);Serial.print("\t");Serial.print("old right encoder :");Serial.println(oldrightEnc);
  Serial.print("new left encoder :");Serial.print(newleftEnc);Serial.print("\t");Serial.print("new right encoder :");Serial.println(newrightEnc);
  
  Serial.print("left differnce  :");Serial.print((newleftEnc - oldleftEnc));Serial.print("\t");Serial.print("left differnce  :");Serial.println((newrightEnc - oldrightEnc));
  Serial.println("--------------");
  Serial.print("total diffrance : ");Serial.println(dist);
  Serial.println("--------------");
  return dist;
  
  }


float goBackward(float sped){

  Adafruit_DCMotor *leftWheel  = AFMS.getMotor(1);   //left motor
  Adafruit_DCMotor *rightWheel = AFMS.getMotor(4);   //right motor

  long oldleftEnc = leftEnc.read();     //reading encoders
  long oldrightEnc = rightEnc.read();

  rightWheel -> setSpeed(sped);
  leftWheel -> setSpeed(sped);
  
  leftWheel -> run(BACKWARD);
  rightWheel -> run(BACKWARD);

  delay(dt);    //10ms is the dt

  long newleftEnc = leftEnc.read();     //reading encoders
  long newrightEnc = rightEnc.read();

  //taking the average of two wheel encoder and then convert to distance in mm ; 1 tick = 0.5mm
  float dist = abs(abs(abs(newrightEnc) - abs(oldrightEnc)) + abs(abs(newleftEnc) - abs(oldleftEnc)))/4;

  return dist;
  


  }

void stopRobot(){  //to stop the robot call dis funcion
  
    Adafruit_DCMotor *leftWheel  = AFMS.getMotor(1);
    Adafruit_DCMotor *rightWheel = AFMS.getMotor(4);

    leftWheel -> run(RELEASE);     
    rightWheel -> run(RELEASE);
    
  
  }
  
//////********************************////////
//Robot driving forward with trapizoidal velocity profile function and trun left and right
///////////***********************///////////////
void moveRobot(char head, int distance){  //get distance in cm 
  
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
      v_curr = min(v_max, v_curr+a_max*(dt-2)); //take the minimum of 
      
      // 1.5mm/ms speed for pwm 200, so coverstion is (200/1.5)*v_curr
      int spedd = 133*v_curr;
      if (head == 'f'){
        diff = goForward(spedd); // run the robot at calculated speed
      }
      else if(head =='b'){
      diff = goBackward(spedd);
      }
      
      }

    else{

      v_curr  = v_curr - a_max*(dt-2);

      int spedd = 133*v_curr;
      
      if (head == 'f'){
        diff = goForward(spedd); // run the robot at calculated speed
      }
      else if(head =='b'){
         diff = goBackward(spedd);
      }
      
      } 

    traveled = traveled + diff;
    Serial.println("/////*****************/////");
    Serial.print("to go distance :"); Serial.println(p_goal);
    Serial.print("traveled :"); Serial.println(traveled);
    Serial.print("distance :"); Serial.println(distance);
    Serial.println("/////*****************/////");
    
    }
    //stop motor after reaching the position
   
    stopRobot();
    
    
    
  
  }


void turnRobot(int dgre, char dir){
  
  long oldLenc =leftEnc.read();
  long oldRenc = rightEnc.read();
  
  float diff = (dgre*240*0.0174533); /// calculate the difference in wheel we need diff = dgree*wheelbase/encoder distance == degree*convert to radian*240mm/0.5mm*
  
  
  
  Adafruit_DCMotor *leftWheel  = AFMS.getMotor(1);   //left motorw
  Adafruit_DCMotor *rightWheel = AFMS.getMotor(4);   //right motor

  rightWheel -> setSpeed(50);
  leftWheel -> setSpeed(50);

  if (dir == 'r'){
  
  leftWheel -> run(FORWARD);
  rightWheel -> run(BACKWARD);
  }
  else if(dir== 'l'){
    leftWheel -> run(BACKWARD);
    rightWheel -> run(FORWARD);
  }
    
    

  float curr_diff = 0;
  long newLenc,newRenc;
  
  
  while(curr_diff <= diff){

    newLenc = leftEnc.read();
    newRenc = rightEnc.read();

    curr_diff = (abs(abs(abs(newLenc)-abs(oldLenc)) + abs(abs(newRenc)-abs(oldRenc))));
    /*
    Serial.print("diff :");
    Serial.println(diff);
    Serial.print("curr_diff :");
    Serial.println(curr_diff);
    
    Serial.println("**************");
    
    Serial.print("old left :");
    Serial.print(oldLenc);
    Serial.print("\t");
    Serial.print("old right :");
    Serial.println(oldRenc);
    
    Serial.print("new left :");
    Serial.print(newLenc);
    Serial.print("\t");
    Serial.print("new right :");
    Serial.println(newRenc);

    Serial.print("diff left :");
    Serial.print(abs(abs(newLenc)-abs(oldLenc)));
    Serial.print("\t");
    Serial.print("diff right :");
    Serial.println(abs(abs(newRenc)-abs(oldRenc)));
    Serial.println("*****************");
    */
    
    
    }
    stopRobot();
   
  
  }

void setup() {

  Serial.begin(115200);
  Serial.flush();
  
  AFMS.begin();

}
///////////**********//////////
//main function statrs here implement I2C or seril handling here. get insturction from serila 
//or i2c and exicute them
///////////*******************////////////


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
    
    
    if(msg[0] == 'f'){   //go forward
    
      int i, dist = 0;
      for (i = 1; i < count; i++){
          dist = 10 * dist + (msg[i]-'0');
      }
     moveRobot(msg[0],dist);
        
    }

    if(msg[0] == 'b'){  //go backward

      int i, dist = 0;
      for (i = 1; i < count; i++){
          dist = 10 * dist + (msg[i]-'0');
      }
      moveRobot(msg[0],dist);
        
    }

    if (msg[0] == 's'){  //stop robot

      stopRobot();
      Serial.println("its stopped");
  
      
      }

    if(msg[0] == 'l'){  //turn left by calling function with direction and angle

      int i, angl = 0;
      for (i = 1; i < count; i++){
          angl = 10 * angl + (msg[i]-'0');
      }
      
      turnRobot(angl,msg[0]);
      
      
      }
    if(msg[0] == 'r'){   //trun right by calling function with direction and angle

      int i, angl = 0;
      for (i = 1; i < count; i++){
          angl = 10 * angl + (msg[i]-'0');
      }
      
      turnRobot(angl,msg[0]);
      
      }
  }

      received =false;
      delay(10);
      Serial.flush();

    
  
  }



 
