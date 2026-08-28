#ifndef AD_F_h
#define AD_F_h

void stand_up(boolean);
void ball_hit();
void init_servo_motors();
void intermittentLogout();
void parseCommand_z(char* data, uint8_t length);
int  getParam(char* data, uint8_t length);
void calculateTarget(int);
void calculateTurn(int);

//odometry
#include  <math.h>
void odometry(float avgspeed);
void odometry_init(int);


/*
int32_t motLeftStep, motRightStep; //+yu
int32_t lastMotLeftStep, lastMotRightStep; //+yu

float dLeft, dRight;
float wx = 0;
float wy = 0;

float wr = 43.0;  //mm  radius
float wD = 115.0; //mm  distance between two wheels
float theta, dtheta;
*/
#endif

//sample patern 
//**** New ****
//
//simple circle patern
//zc 0 90 500 720 0 -90
//zc 0 90 500 340 0 -80   //微調整
//zc 450 720 0 110  //double
//zc 450 690 0 120

//square patern, turnt left, 500mm each side
//zc 10000 500 0 85 10000 500 0 85 10000 500 0 85 10000 500 0 85 
//  double 70mm
//zc 10000 700 0 85 10000 700 0 85 10000 700 0 85 10000 700 0 85 10000 700 0 85 10000 700 0 85 10000 700 0 85 10000 700 0 185
//  double 650mm
//zc 10000 650 0 85 10000 650 0 85 10000 650 0 85 10000 650 0 85 10000 650 0 85 10000 650 0 85 10000 650 0 85 10000 650 0 185

//spiral patern, turn left, diameter is 1000
//* looking at the camera at start and end
//zc 0 90 500 360 400 360 300 360 200 360  0 270
//zc 500 180 450 189 400 180 350 180 300 180 250 180 200 180 150 180  0 100
//zc 10000 650 0 83 10000 650 0 83 10000 650 0 83 10000 650 0 83 10000 650 0 83 10000 650 0 83 10000 650 0 83 10000 650 0 90

//
//slalom1  looking at the camera
//zc 0 -135 200 -180 200 180 200 -360 200 180 200 -180 0 135
//zc 0 90  200 -180 200 180 200 -180 200 370 200 -180 200 180 200 -180 0 -145
//slalom2 looking at the camera
//zc 0  -180  200 -90 200 90 200 -90 200 90 200 -270 200 90 200 -90 200 90  200 -180 0 -180
//zc 0  -180  200 -90 200 90 200 -90 200 90 200 -270 200 90 200 -90 200 90  200 -90 0 -270
//slalom3 Not tested yet
//zc 200 180 10000 200 200 -90 10000 400 200 90 10000 200 200 90 10000 200 200 90 10000 400 200 -90 10000 200
//slalom3-2
//zc 0 -180 200 -90 200 90 200 -90 200 90  10000 200 200 -270 10000 200 200 90 200 -90 200 90 200 -90 0 90 
//zc 0 45 200 -90 200 90 200 -90 200 90  10000 200 200 -270 10000 200 200 90 200 -90 200 90 200 -90 0 180

//spiral
//zc 0 90 500 90 450 90 400 90 350 90 300 90 250 90 200 90 150 360 0 360 
//zc 500 90 450 90 400 90 350 90 300 90 250 90 200 90 150 360 0 530
//
//oval patern
//zc 0 90 10000 500 250 180 10000 500 250 180 0 260
//zc 10000 500 400 180 10000 500 400 180 0 90
//cross patern
//zc 0 60 10000  433 250 240 10000 866 250 -240 10000 433 0 -60
//zc 300 360 300 -360 300 360 300 -360 

//**** Old ****
// square pattern  0.5m and 1.0m, start to front direction
//zc 10000 500 0 90 10000 500 0 90 10000 500 0 90 10000 500 0 90 
//zc 10000 500 100 90 10000 500 100 90 10000 500 100 90 10000 500 100 90
//zc 10000 1000 100 90 10000 1000 100 90 10000 1000 100 90 10000 1000 100 90 

// simple circle  0.3m and 0.4m
//zc 150 360 200 360

//sumple command patterns
//slalom1  right direction
//zc 100 180 -100 180 100 360 -100 180 100 180
//slalom2  front direction
//zc 0 -90 100 180 -100 180 100 360 -100 180 100 180 0 90


// slalom with round trip, r=100, 200, 150, 250, 300: front direction
//zc 0 -90 100 180 -100 180 100 180 -100 180 100 360 -100 180 100 180 -100 180 100 180 0 90
//zc 0 -90 200 180 -200 180 200 360 -200 180 200 180 0 90
//zc 0 -90 150 180 -150 180 150 180 -150 180 150 360 -150 180 150 180 -150 180 150 180 0 90 
//zc 0 -90 250 180 -250 180 250 360 -250 180 250 180 0 90 
//zc 0 -60 300 120 -300 120 300 300 -300 120 300 120  0 150

//decreasing three tircle
//zc 0 -90 500 360 300 360 150 360 0 90 

//spiral patern
//zc 0 -90 500 90 450 90 400 90 350 90 300 90 250 90 200 90 150 90 100 90  0 90 
//spiral and spin
//zc 100 -90 500 90 450 90 400 90 350 90 300 90 250 90 200 90 150 90 100 90  50 90 10 360 0 360

//square patern
//zc 10000 1000 0 90 10000 01000 0 90 10000 1000 0 90 10000 1000 0 90

//flag patern
//zc 10000 500 100 90 10000 500 -100 90 10000 500 -100 90 10000 500 -100 90 10000 1000
//