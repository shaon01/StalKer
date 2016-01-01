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


///////*******************************///////////
//infinite driving function
///////******************************////////////

void infDrive(char drv){

  Adafruit_DCMotor *leftWheel  = AFMS.getMotor(1);   //left motor
  Adafruit_DCMotor *rightWheel = AFMS.getMotor(4);   //right motor

  int frd = 40;
  int trn = 20;


  switch (drv){
      

        case 'w':    //go forward
        
        rightWheel -> setSpeed(frd);
        leftWheel -> setSpeed(frd);
        
        leftWheel -> run(FORWARD);
        rightWheel -> run(FORWARD);
        break;

        case 'z':     //go backward

        rightWheel -> setSpeed(frd);
        leftWheel -> setSpeed(frd);
        
        leftWheel -> run(BACKWARD);
        rightWheel -> run(BACKWARD);
        break;

        case 'a':    //left turn

        rightWheel -> setSpeed(trn);
        leftWheel -> setSpeed(trn);
        
        leftWheel -> run(BACKWARD);
        rightWheel -> run(FORWARD);
        break;
        
        case 'd':     //right turn

        rightWheel -> setSpeed(trn);
        leftWheel -> setSpeed(trn);
        
        leftWheel -> run(FORWARD);
        rightWheel -> run(BACKWARD);
        break;

        
      
  }

  
  
  
  }

//////////****************************///////////////
//Basic motor driving function
///////////////**********************///////////


void goForward(int sped, char hed){  /// to run the robot forward call this function
  
  Adafruit_DCMotor *leftWheel  = AFMS.getMotor(1);   //left motor
  Adafruit_DCMotor *rightWheel = AFMS.getMotor(4);   //right motor

  rightWheel -> setSpeed(sped);
  leftWheel -> setSpeed(sped);

  if (hed == 'f'){        //for forward movement
    leftWheel -> run(FORWARD);
    rightWheel -> run(FORWARD);
  }
  if (hed == 'b'){             //for backward movement
    leftWheel -> run(BACKWARD);
    rightWheel -> run(BACKWARD);
  }

  delay(dt);    //20ms is the dt
  
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

  long oldleftEnc = leftEnc.read();     //reading encoders
  long oldrightEnc = rightEnc.read();

   while((p_goal - traveled)>0){

    dp_min = (0.5*v_curr*v_curr)/a_max;  //critical distance to strat breaking
    toGo = p_goal - traveled;            // how far to go
 
    if(dp_min<toGo){   //increase speed
      
      v_curr = min(v_max, v_curr+a_max*(dt-2));   //take the minimum of so it does not exceed max speed 
            
      int spedd = 133*v_curr;     // 1.5mm/ms speed for pwm 200, so coverstion is (200/1.5)*v_curr
      
      goForward(spedd, head); // run the robot at calculated speed and heading
      
      
      }

    else{

      v_curr  = max(0, v_curr - a_max*(dt));

      int spedd = 133*v_curr;
      
      goForward(spedd, head); // run the robot at calculated speed
      
      
      } 
     long newleftEnc = leftEnc.read();     //reading encoders
     long newrightEnc = rightEnc.read();

     if (head == 'f'){
        traveled = (((newrightEnc - oldrightEnc) + (newleftEnc - oldleftEnc))/2)*0.25;    //convert average of encoder to mm. 1 tick = 0.25mm
     }
     else {
        traveled = abs(abs(abs(newrightEnc) - abs(oldrightEnc)) + abs(abs(newleftEnc) - abs(oldleftEnc)))/4; ////convert average of encoder to mm. 1 tick = 0.25mm
     }

    
   /*
    Serial.print("old left encoder :");Serial.print(oldleftEnc);Serial.print("\t");Serial.print("old right encoder :");Serial.println(oldrightEnc);
    Serial.print("new left encoder :");Serial.print(newleftEnc);Serial.print("\t");Serial.print("new right encoder :");Serial.println(newrightEnc);
    
    Serial.print("left differnce  :");Serial.print((newleftEnc - oldleftEnc));Serial.print("\t");Serial.print("left differnce  :");Serial.println((newrightEnc - oldrightEnc));
    Serial.println("--------------");
    
    
   
    Serial.print("to go :"); Serial.println(toGo);
    Serial.print("dp_min :"); Serial.println(dp_min);
    Serial.print("speed :"); Serial.println(v_curr);
     
    Serial.println("/////*****************/////");*/
    
    }
    //stop motor after reaching the position
   
    stopRobot();
    
    
    
  
  }


void turnRobot(char dir, int dgre){
  
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
    Serial.print("diff :");Serial.println(diff);
    Serial.print("curr_diff :");Serial.println(curr_diff);
    
    Serial.println("**************");
    
    Serial.print("old left :");Serial.print(oldLenc);Serial.print("\t");Serial.print("old right :");Serial.println(oldRenc);
    Serial.print("new left :");Serial.print(newLenc);Serial.print("\t");Serial.print("new right :");Serial.println(newRenc);

    Serial.print("diff left :");Serial.print(abs(abs(newLenc)-abs(oldLenc)));Serial.print("\t");Serial.print("diff right :");Serial.println(abs(abs(newRenc)-abs(oldRenc)));
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
  int count = 0,val = 0;
  
  while (Serial.available() > 0) {

    byte inByte = (byte)Serial.read(); //reading serial
    msg[count] = inByte;

    count++;
    received = true;
   
  }

  if (count > 1){  ///converting byte to a integer
    int i;
      for (i = 1; i < count; i++){
          val = 10 * val + (msg[i]-'0');
          
      }
    
    }
 
  
  if(received){

    switch (msg[0]){
      

        case 'f':
        moveRobot(msg[0],val);
        Serial.write('d');
        break;

        case 'b':
        moveRobot(msg[0],val);
        Serial.write('d');
        break;

        case 'l':
        turnRobot(msg[0],val);
        Serial.write('d');
        break;
        
        case 'r':
        turnRobot(msg[0],val);
        Serial.write('d');
        break;

        case 's':
        stopRobot();
        Serial.println("its stopped");
        break;

        case 'w':
        infDrive(msg[0]);
        break;

        case 'z':
        infDrive(msg[0]);
        break;

        case 'a':
        infDrive(msg[0]);
        break;

        case 'd':
        infDrive(msg[0]);
        break;
      
      
      }
   
  }

      received =false;
      delay(10);
      Serial.flush();

    
  
  }



 
