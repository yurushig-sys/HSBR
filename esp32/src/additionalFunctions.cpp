//yu original, additional functions

//#include <Arduino.h>
#include <Streaming.h>
//#include "additionalFunctions.h"
#include <ESP32Servo.h>
#include <math.h>
#include <string.h>
#include <Preferences.h> 
#include <PID.h>
#include "additionalFunctions.h"
#include <fastStepper.h>

extern Preferences preferences;
extern PID pidAngle, pidPos, pidSpeed;
extern fastStepper motLeft, motRight;
//
//odeometry
//
extern uint8_t microStep;
extern int32_t motLeftStep, motRightStep, lastMotLeftStep, lastMotRightStep;
extern float motLeftSpeed, motRightSpeed;
extern float theta, wx, wy, lp;
extern float steerInput, speedInput;
const float wr = 39.0; //38.0; //43.0;  //mm  radius
const float wD = 115.0; //mm  distance between two wheels
float maxSpeedInput = 80; //50; //20;  //100
float initSpeedInput = 50, minSpeedInput = 40; //30;
float targetSpeed = maxSpeedInput;
float maxTurnSpeed = 60;
int rotateFlag, moveFlag, turnFlag, turnFlag2, continuousFlag, movingFlag;
//int moveCount;
float turnSteer, targetTurnSteer;
float TspeedInput, tempSpeedIn;
float positionP = 0.5;

//Stand Up control
extern int delay_counter, standupControl, ballhit;
extern float accAngle, filterAngle, deltaGx;
extern boolean enableControl;
int faceUp, faceDown; 

extern Servo rightservo1;  // Right, Low  Stand up Stick     
extern Servo leftservo1;   // Left, Low   Hitting Stick
extern Servo rightservo2;  // Right. Low  Camera H
extern Servo leftservo2;   // Left, Low   Camera V

void stand_up(boolean enableControl){
  static int face_up, face_down;
  static float stick_right; //, stick_left;
    
  delay_counter += 1;
  if (delay_counter <= 1){
    Serial.println("stand up start");
    stick_right = 90;
    rightservo1.write(stick_right); 
    face_up = 0; face_down = 0;
    if(accAngle < -45) face_up = 1;           //face down
    else if(accAngle > 45) face_down = 1;
    return ;
  }
  if (delay_counter < 200) return;  //delay 1.0 sec

  if (delay_counter == 200){
    if(face_down  == 1)  stick_right = 0;   //10; 
    else if (face_up == 1)  stick_right = 180;   //10;
    rightservo1.write(stick_right); 
    return ;
  }
  if (delay_counter > 400 &&  (delay_counter % 2 == 0)){
      if(face_down  == 1) {                   //face down
        if (stick_right < 120) stick_right += 10.0;
        if (enableControl == 1) {
          face_down = 0;
          faceDown = 1; faceUp = 0;
          standupControl = 0;
          delay_counter = 0;
        }
      }
      else if (face_up == 1){
        if (stick_right > 60) stick_right -= 10.0;
        if (enableControl == 1) {
          face_up = 0;
          faceUp = 1; faceDown = 0;
          standupControl = 0;
          delay_counter = 0;
        }
      }
      else {
        faceUp = 0;
        faceDown = 0;
      }
      rightservo1.write(stick_right); 
      //Serial << stick_right << " " << enableControl << endl;
  }
  if (delay_counter > 1000){
    standupControl = 0;
    delay_counter = 0;
  }
}

void ball_hit(){
  delay_counter += 1;
  if (delay_counter <= 1){
    //leftservo1.write(50);
    leftservo1.write(115);
  }
  if (delay_counter < 100) return; //delay 0.5sec
  if (delay_counter == 100){
     //leftservo1.write(150);
     leftservo1.write(65);
  }
  if (delay_counter < 250) return; //delay 0.5sec
  if (delay_counter >= 250){
     leftservo1.write(90);
     ballhit = 0;
     delay_counter = 0;
  }
}

//Initialize servo motors position

void init_servo_motors(){
  rightservo1.write(90);   //right low, standup stick
  leftservo1.write(90);    //left low, hitting stick
  rightservo2.write(170); //(170);  //right up, Camera Horizontal (right)20-(left)170
  leftservo2.write(90);  //85);    //left up,  Camera Vertical, up(22)-down(170)
  //delay(500);
  //rightservo1.write(0);    //stop servo(MG92B)
  //leftservo1.write(0);     //stop servo(MG92B)
  }


//intermittent Log out
extern float avgBatteryVoltage;
extern float deltaGyroAngle;
#define battLowPin 18 

void intermittentLogout(){
  static int intervalTimer;
  intervalTimer += 1;
  if ((intervalTimer % 20000) == 0){  //5ms * 2000 = 10sec
    if (digitalRead(battLowPin) == 0){
      Serial << "BATTERY LOW: " ;
    }
    Serial << "Battery Voltage =" << avgBatteryVoltage 
    << " AccAngle =" << accAngle
    << " DeltaGAngle =" << deltaGyroAngle
    << " FilterAngle =" << filterAngle
    << endl; 
  }
}

//
//parse Command z
//
extern int logena;
extern float theta, wx, wy, lp;
extern float Ttheta, Twx, Twy, Twx2, Twy2, Tlp;
extern uint8_t boardNumber ;
extern float gyroCompensate, angleOffset;
extern int standupControl, falldownControl, traceon, rcwSpeedOn;
const int paramMax = 64;
char param[paramMax][16];
char paramTemp[paramMax][16];
int pn, pnTemp, paramIndex, hit, hitPn;
float runLength, hittingPoint, hittingTime;
extern float dT;

void parseCommand_z(char* data, uint8_t length){

  pn = getParam(data, length);
  
  //Serial << "Number of Params = " << pn <<endl;

  switch (data[1]) {
    case 'a' :   //set angle_offset to flush
      Serial << "Updating angle offset from " << angleOffset;
      angleOffset = atof(param[0]);
      Serial << " to " << angleOffset << endl;
      preferences.putFloat("angle_offset", angleOffset);
      break;

    case 'b' :        //set Board Number
      boardNumber = atoi(data+2);
      Serial << "board number = " << boardNumber <<endl;
      preferences.putUInt("board_No.", boardNumber);
      if (pn == 1){
        gyroCompensate = atof(param[0]);
        preferences.putFloat("gyroCompensate", gyroCompensate);
        Serial << "gyroCompensate = " << gyroCompensate <<endl;
      }
      break;

    case 'e':
      logena = atoi(data+2);
      break;

    case 'r' :       //reset odometery (set current position to zero)
      odometry_init(atoi(data+2));
      pidAngle.reset();
      pidPos.reset();
      pidSpeed.reset();
      if (pn == 4){
        wx = atof(param[0]);
        wy = atof(param[1]);
        lp = atof(param[2]);
        theta = atof(param[3]) * M_PI/180;
      }
      else {
        wx = 0;
        wy = 0;
        theta = 0;
      }
      Serial <<"wx="<< wx <<" wy=" << wy 
             <<" lp=" <<lp<<" theta="<<theta * 180/M_PI
             << endl;
      break; 

    case 'h' :        //Halt all movement
      moveFlag = 0;
      rotateFlag = 0;
      turnFlag = 0;
      turnFlag2 = 0;
      continuousFlag = 0;
      speedInput, tempSpeedIn = 0, 0;
      steerInput = 0; 
      break;
    
    case 'j' :        // joystick control from Raspberry Pi
      if (pn == 2){
        speedInput = atof(param[0]);
        steerInput = atof(param[1]);
        Serial << "speedInput=" << speedInput 
               << " steerInput=" << steerInput 
               << endl;
      }
      else {
        Serial << "parameter number error, n=" << pn << endl;
      }
      break;


    case 'u' :       //stand up or fall down
      if (atoi(data+2) == 1 && standupControl == 0){
        standupControl =1 ;
        falldownControl = 0;
        Serial<<"standupControl=1"<<endl;
      }
      else if (atoi(data+2) == 0 ) {
        standupControl = 0;
        falldownControl = 1;
      } 
      break;

    case 't' :        //set target x, y position
      /*
      speedInput = atof(param[0]);  //-100 ~ +100
      steerInput = atof(param[1]);  //^50 ~ +50
      Serial << speedInput << "\t" << steerInput << endl;
      */
      if (pn == 2 || pn == 4){
        //Serial << "Target x = " <<atof(param[0]) << " Target y = " << atof(param[1]) << endl;
        Twx = atof(param[0]);
        Twy = atof(param[1]);
        if (pn == 2){
          calculateTarget(1);     //One target
        }
        else {                    //Two targets
          Twx2 = atof(param[2]);
          Twy2 = atof(param[3]);
          calculateTarget(2);
        }
      }
      else {
        Serial << "parameter number error, n=" << pn << endl;
      }
      break;

    case 'o' :        //set relative rotate angle (degree)
      if (pn >= 1){
        Ttheta = theta + atof(param[0]) * M_PI /180 ;
        rotateFlag = 1;
      }
      if (pn == 2) maxTurnSpeed = atof(param[1]);
      break;

    case 'O' :        //set absolute rotate angle (degree)
      if (pn >= 1){
        Ttheta = atof(param[0]) * M_PI /180 ;
        rotateFlag = 1;  
      }
      if (pn == 2) maxTurnSpeed = atof(param[1]);
      break;

    case 'd' :        //get data 
      if (atoi(data+2) == 0){
        Serial << "Status " << enableControl <<" "<< moveFlag 
        <<" "<< rotateFlag <<" "<< continuousFlag<<" "<< movingFlag<<" "<< paramIndex <<endl;
      }
      else if (atoi(data+2) == 1){
        Serial << "Data " << wx <<" "<< wy <<" "<< lp <<" "<< Tlp <<" "
                <<theta * 180/M_PI<<" "<<filterAngle<<" "<<avgBatteryVoltage<<endl;
                //<<theta * 180/M_PI<<" "<< deltaGx*10 <<" "<<avgBatteryVoltage<<endl;
      }
      //Serial <<"wx="<<wx<<" wy="<<wy<<" theta="<<theta * 180/M_PI
      //       << " lp="<<lp<<" Tlp="<<Tlp<<endl;
      break;
    case 'c' :      // turn around specified center
      if (pn > paramMax){
         Serial << "parameter error n=" << pn << endl;
         break;
         }
      if (pn % 2 == 1 &&  atof(param[pn-1]) != 0){
        hittingPoint = atof(param[pn-1]);
        Serial << "Hitting Point = " << hittingPoint << endl;
        hit = 1;
        pn -= 1;
        hitPn = pn;
      } 
      else {
        hittingPoint = 0;
        hit = 0;
      }
      pnTemp = pn;
      for (int i = 0; i < pnTemp ; i++) strcpy(paramTemp[i], param[i]);
      paramIndex = 0;
      calculateTurn(paramIndex);
      continuousFlag = 1;
      runLength = 0;
      break;
      
    case 'p':     //set position x
      Tlp = atof(param[0]); //[mm]
      lp = 0;
      moveFlag = 1;
      targetSpeed = maxSpeedInput;
      break;
    
    case 'x':      //set parameter
      maxSpeedInput = atof(param[0]);
      initSpeedInput = atof(param[1]);
      Serial << "maxSpeedInput=" << maxSpeedInput 
             << " initSpeedInput=" << initSpeedInput 
             << endl;
      break;
    case 's':     //Servo Control
      switch (data[2]){
        case 'i':   //init all servo
          init_servo_motors();
          break;
        case 'h':   //camera Horizontal 
          rightservo2.write(atoi(param[0]));
          Serial << "camera H=" << atoi(param[0]) << endl;
          break;
        case 'v':   //camera Vertical
          leftservo2.write(atoi(param[0]));
          Serial << "camera V=" << atoi(param[0]) << endl;
          break;
        case 's':   //hitting stick
          leftservo1.write(atoi(param[0]));
          Serial << "ball stick=" << atoi(param[0]) << endl;
          break;
      }
      break;

    case 'Y':    //Over weite PID parameters
      if (pn == 4){
        switch (data[2]){
          case 'A':  //Angle PID
            pidAngle.setParameters(atof(param[0]), atof(param[1]), atof(param[2]), atof(param[3]));
            Serial <<"Angle P="<< param[0] <<" I=" << param[1] <<" D=" << param[2] <<" N="<< param[3] << endl;
            break;   
          case 'P':  //Pos PID
            pidPos.setParameters(atof(param[0]), atof(param[1]), atof(param[2]), atof(param[3]));
            Serial <<"Pos P="<< param[0] <<" I=" << param[1] <<" D=" << param[2] <<" N="<< param[3] << endl;
            break;  
          case 'S':  //Speed PID
            pidSpeed.setParameters(atof(param[0]), atof(param[1]), atof(param[2]), atof(param[3]));
            Serial <<"Speed P="<< param[0] <<" I=" << param[1] <<" D=" << param[2] <<" N="<< param[3] << endl;
            break;
        }
      }
      else if (pn == 1){
        positionP = atof(param[0]);
        Serial << "positionP =" << positionP << endl;
          /*
          Serial <<"Angle P="<< param[0] <<" I=" << param[1] <<" D=" << param[2] <<" N="<< param[3] << endl;
          Serial <<"Pos P="<< param[0] <<" I=" << param[1] <<" D=" << param[2] <<" N="<< param[3] << endl;
          Serial <<"Speed P="<< param[0] <<" I=" << param[1] <<" D=" << param[2] <<" N="<< param[3] << endl;
          */
        break;
      }
      break;
  
  }
}

int getParam(char* data, uint8_t len){
  char *m, *n;
  int pn_ = 0;

  m = strchr(data, ' ') ; //find 1st space
  if ( m == NULL){
    return 0;
  }
  m += 1;
  while (true) {
    while (*m == ' ') {
      m ++; //remove continuous space
    }
    if (*m == '\0') return pn_;

    n = strchr(m, ' ');  //find 2nd space
    if (n == NULL){
      strncpy(param[pn_], m, len - (m - data));
      return pn_ + 1;
    }
    else {
      strncpy(param[pn_], m, n-m);
      param[pn_][n-m] = '\0' ;
      m = n + 1;
      pn_ += 1; 
      if (pn_ > paramMax) {
        Serial << "over parameters, paramNumber="<< pn_
               << "max Param=" << paramMax << endl;
      }
    }
  }
}


//
//odeometry
//
float dBT, turnTheta;
float dAverage, aveSum, aveSpeed;
int aveCount, maxCount = 50;
void odometry(float avgspeed){
  float dLeft, dRight; //, dAverage;
  float dtheta, d_theta;
  static int running = 0;

  //calculate current position 
  dLeft = wr * 2 * M_PI * (motLeftStep - lastMotLeftStep) / (microStep * 200);
  dRight = wr * 2 * M_PI * (motRightStep - lastMotRightStep) / (microStep * 200);
  
  dAverage = dRight + (dLeft - dRight) / 2;
  dtheta = atan((dLeft - dRight) / wD);
  theta -= dtheta;
  runLength -= dAverage;
  aveSum += dAverage;
  aveCount += 1;
  if (aveCount == maxCount){
    aveSpeed = aveSum / (maxCount * dT * 1000);  //[m/s]
    aveSum = 0;
    aveCount = 0;
    if (abs(aveSpeed) <  0.05) movingFlag = 0;
    else movingFlag =1;
    //Serial << "aveSpeed=" << aveSpeed << "[m/s]  " << movingFlag
    //       <<" "<<moveFlag<<" "<<rotateFlag<< endl;
  }

  wx = wx - dAverage * cos(theta);
  wy = wy - dAverage * sin(theta);
  lp = lp - dAverage;
  lastMotLeftStep = motLeftStep;
  lastMotRightStep = motRightStep;

  //rotate control 
  if (rotateFlag == 1){
    d_theta = Ttheta - theta;
      //0.017[radian] = 1.0[degree]
    if (d_theta < -0.017 ) steerInput += 1.0; //40.0; //if (d_theta < -1.0[degree]) rotate right
    else if (d_theta > 0.017) steerInput += -1.0; //-40.0; //if (d_theta > 1.0[degree]) rotate left
    if (steerInput > maxTurnSpeed) steerInput = maxTurnSpeed;
    else if (steerInput < -maxTurnSpeed) steerInput = -maxTurnSpeed;
    //Serial << steerInput << " " << d_theta <<endl;
    if (d_theta < 0.017 && d_theta > -0.017) {
      steerInput = 0;
      rotateFlag = 0;
      Serial << "theta=" << theta * 180/M_PI << endl;
    }
    //Serial <<speedInput<<" "<<steerInput<<endl;
  }

  //move control
  float pError;

  if (moveFlag > 0 && rotateFlag == 0){
    pError = Tlp - lp;
    if (moveFlag == 1 && abs(pError) >=  50) {     //acceleration phase
      if (abs(tempSpeedIn) < maxSpeedInput) 
        if (Tlp > 0) tempSpeedIn +=  1.0; //0.25;
        else tempSpeedIn -= 1.0; //0.25;
      else moveFlag = 2; //constant speed
      speedInput = tempSpeedIn;
    }
    else if ( moveFlag == 2){
      if (abs(pError) < 50) moveFlag = 3; //deceleration phase
    }
    else if (moveFlag == 3){
      //if (continuousFlag == 0){
        //if (Tlp > 0){
        //  if (pError > 0) speedInput = tempSpeedIn * 1.2; //breaking;
        //  else moveFlag = 4;
        //}
        //else {
          if (pError < 0) speedInput = tempSpeedIn * 0.5; //breaking;
          else moveFlag = 4;
        //}
      //}
      //else moveFlag = 4;
    }
    else if (moveFlag ==4){
        if (continuousFlag == 0) {
          if (speedInput > 0) speedInput -= 10;
          else {speedInput = 0;
            tempSpeedIn = 0;
            moveFlag = 0;
          }
        }
        else {
          moveFlag = 0;
        }
        //speedInput = 0;
        //tempSpeedIn = 0;
        //moveFlag = 0;
      }
    /* -----? */ 
    Serial <<"mF="<<moveFlag<<" pEr="<<pError
           <<" spIn="<<speedInput
           <<" Angle="<<filterAngle
           <<endl;/* <----- */
    if (moveFlag == 1 && abs(pError) < 50){
      speedInput = 0;
      tempSpeedIn = 0;
      moveFlag = 0;
    }
  }

  //continuous turn control;
  if (continuousFlag == 1){
    if (turnFlag2 == 1){
      d_theta = Ttheta - theta;
      // if (tempSpeedIn > targetSpeed) tempSpeedIn -= 10.0; //0.5;
      // if (tempSpeedIn < targetSpeed) tempSpeedIn += 10.0; //0.5;
    
      if ((turnTheta >= 0 && d_theta < 0.0) || (turnTheta <0 && d_theta > 0.0)) {
        if (turnSteer < 1.0){
          turnSteer += 0.2; //0.04;
          if (turnSteer > 1.0) turnSteer = 1.0;
        } 
        else {
          steerInput = 0; 
          if (continuousFlag == 0) tempSpeedIn = 0;
          turnFlag2 = 0;
          turnSteer = 0;
          Serial << "theta=" << theta * 180/M_PI << endl;
          }
      }
      else{
        if (turnSteer > targetTurnSteer) turnSteer -= 0.01;
      }
      Serial <<"dTh="<<d_theta
           <<" spIn="<<speedInput
           <<" turnSteer="<<turnSteer
           <<endl; 
    }
    else if (moveFlag == 0 && rotateFlag == 0){
      pnTemp -= 2;
      if (pnTemp > 0){
        paramIndex += 2;
        //if (paramIndex == 6) leftservo1.write(150);
        calculateTurn(paramIndex);
        //if (hit == 1 && pnTemp == hitPn ){
        //  leftservo1.write(120);
        //  hit = 0;
        //}
      }
      else {
        if (hit == 1) {
          leftservo1.write(120);
          Serial << "Hit!!" <<  endl;
          hit = 0;
        }
        tempSpeedIn = 0;   //<--20240108
        speedInput = 0;    //<--20240108
        steerInput = 0;    //<--20240108
        turnFlag = 0;
        turnFlag2 = 0;
        //odometry_init(1);  //<--20240108
        continuousFlag = 0;
      }
    } 
    //Serial <<"dAverage="<<dAverage<<" runLength="<<runLength<<endl;
    /*if (dAverage*(-1) > 0){
      hittingTime = (hittingPoint - runLength)/(abs(dAverage)/dT);
      if (hittingTime < 0.18 && hit == 1) {
        leftservo1.write(120);
        hit = 0;
        Serial << "Hit at " << runLength 
            << " " <<hittingTime <<" "<< hittingPoint<< endl;
      }
    }*/
    //speedInput = tempSpeedIn;
    //Serial <<speedInput<<" "<<steerInput<<endl;
  }
}


void calculateTarget(int tn){
  Tlp = sqrt(pow((Twx - wx), 2) + pow((Twy - wy),2)); //get target distance

  //get target theta (0 - 2pai = 0 - 360
  if (Tlp != 0) Ttheta = acos((Twx - wx)/Tlp);
  else Ttheta = theta;
  if (Twy - wy < 0) Ttheta *= -1;
  Serial <<"wx="<<wx<<" wy="<<wy<<" theta="<<theta<<" Twx="<<Twx<<" Twy="<<Twy
         <<" Tlp="<<Tlp<<" Ttheta="<<Ttheta*180/M_PI << endl;
  rotateFlag = 1;
  moveFlag = 1;
  lp = 0;
}

void calculateTurn(int n){

  dBT = atof(paramTemp[n]); //distance between center of body and center of turn
                            //if (+) right turn else left turn
                            //if (|dBT| >= 10000.0) go to straight
  turnTheta = atof(paramTemp[n+1]) ; //if (|dBT| <100000) turning theta [degree]
                                     // else straight distance [mm]

  //if (abs(dBT) < wD/2) dBT = wD/2; 
  //turnSteer = (abs(dBT) - wD/2)/(abs(dBT) + wD/2);
  turnSteer = 1.0;
  targetTurnSteer = (abs(dBT) - wD/2)/(abs(dBT) + wD/2);
  
  //maxSpeedInput = (abs(dBT - wD/2) * 0.1 + 40); //50.0);
  if (dBT >= 60){
    targetSpeed = dBT * 1.0; // 0.2; //0.3; //0.4; // 0.5 ; //value is decided by expriment
    if (targetSpeed > maxSpeedInput ) targetSpeed = maxSpeedInput;
    initSpeedInput = targetSpeed/2;    //<-- 20240108 20240113
    //initSpeedInput = targetSpeed * 0.7;  //<-- 20240108
  } 
  else{
    targetSpeed = 10; //20;
    initSpeedInput = 0;
  } 
  speedInput = initSpeedInput ;
  Ttheta = theta + turnTheta * M_PI/180;
  //if (Ttheta > M_PI * 2) Ttheta -= M_PI * 2;
  Serial <<turnTheta<<" "<<theta*180/M_PI<<" "<<Ttheta*180/M_PI
         <<" "<<turnSteer<<endl;
  if (dBT < 60) {
    rotateFlag = 1;
    speedInput = 0;
    tempSpeedIn = 0;
    targetSpeed = 0;
    steerInput = 0;
  }
  else if (dBT < 10000) {
    //turnFlag = 1;
    turnFlag2 = 1;
    //Serial << Ttheta*180/M_PI << " " << theta*180/M_PI << endl;
  }
  else{
    Tlp = turnTheta;
    moveFlag = 1;
    lp = 0;
  }
  //turnFlag = 1;
  //Serial << "paramIndex=" << n << "speedInput="<<speedInput<<endl;
  Serial << rotateFlag << " " << turnFlag2 << " " << moveFlag <<endl;
}
  

void odometry_init(int rst){    //rst==0: reset, rst==1:stop
  //theta = 0;
  //wx = 0;
  //wy = 0;
  lp = 0;
  Tlp = 0;
  Ttheta = 0;
  moveFlag = 0;
  rotateFlag = 0;
  turnFlag = 0;
  turnFlag2 = 0;
  continuousFlag = 0;
  if (rst == 0)speedInput = 0;
  steerInput = 0;
  motLeftStep = 0;
  motRightStep = 0;
  motLeft.setStep(0);
  motRight.setStep(0);
  lastMotLeftStep = motLeftStep;
  lastMotRightStep = motRightStep;
  
  aveSum = 0;
  aveSpeed = 0;
  aveCount = 0;
  movingFlag = 0;
}