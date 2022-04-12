#include <Wire.h>
#include <VL6180X.h>
#include <Math.h>
#include "motor.h"
#include "encoders.h"
#include "kinematics.h"

#define PI 3.1415926535897932384626433832795
#define SCALING 2
#define FinalX 1000
#define FinalY 1500

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

Kinematics pose;
VL6180X sensor;

int state;                               /* run different functions*/
bool Flag;                               /* global variables*/
float initial_theta;                     /* starting position - angle*/
unsigned long current_time;              /* local time*/
unsigned long prev_time;                 /* timestamp*/
float Dy = FinalY - pose.get_ypos();     /* position differences*/
float Dx = FinalX - pose.get_xpos();
float distance;                          /* distance between local position and destination*/
float theta = pose.get_theta();          /* local position - angle*/
float speed;                             /* turning velocity*/
float power = 15.0f;                     /* straight velocity*/
float TurnAngle;                         /* from local angle to dest.angle*/

float xpose = pose.get_xpos();           /* local position*/
float ypose = pose.get_ypos();

int F_cr = 10;                          //试用值
int F_ct = 1;
float phi = 0.9;                         /* Installation rad */
long F_rp_x;                        /* Initialize force */
long F_rp_y ;
float F_at_x = 0;
float F_at_y = 0;
float F_x;
float F_y;
float theta1;
//float jiaodu[10000];

void setup() {
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  setupEncoder0();                       /* initialize encoder*/
  setupEncoder1();
  pose.setPose( 0, 0, 0 );
  Flag = 0;
  state = 1;
  theta = 0;
  xpose = 0;
  ypose = 0;
  Dx = 0;
  Dy = 0;
  TurnAngle=0;
  prev_time = millis();
  theta1=0;
  Serial.begin( 9600 );                  /* initialize system*/
  delay(1000);

  Wire.begin();                          /* initialize Wire*/
  sensor.init();                         /* initialize I2C*/
  sensor.configureDefault();
  sensor.setScaling(SCALING);
  sensor.setTimeout(500);

}


void loop() {
  long sensor_distance = sensor.readRangeSingleMillimeters();  /* sensor feedback from VL6180X*/
  unsigned long current_time = millis();
  unsigned long update_time = current_time - prev_time;

  if (update_time > 5) {
    pose.update( e0_count, e1_count);  /* update the kinematics*/
    theta = pose.get_theta();
    //Serial.println (theta);
    xpose = pose.get_xpos();           /* local position*/
    ypose = pose.get_ypos();
       //Serial.println (  xpose);
    //char heng[8];
    //char zong[8];
    //sprintf(heng,"%s",xpose);
    //sprintf(zong,"%s",ypose);
    int t;                                                     // sensor turning angle.
    t =  PI / 2 + theta + phi;                                 // phi (unit: degree) is angle between sensor and central line.
    int x_i =  xpose + sensor_distance * cos(t);               // cordinates of deteced distance.
    int y_j = ypose + sensor_distance * sin(t);

    certainty_cal(x_i, y_j,t);                         // return C[i][j]
    Force(F_rp_x, F_rp_y, xpose, ypose, theta);                      // return theta1
    // sensor_reading(); return x_ i,y_j for Force()
      //Serial.println ( F_rp_x);
     F_rp_x=0;//排斥力
      F_rp_y=0;
    
    Change(theta1);
     // Serial.println (theta1);
   printdata();
    if ((ypose == FinalY) && (xpose == FinalX)) {
      Stop();
    }
  }
}

void Stop() {
  leftMotor(0);
  rightMotor(0);
}



/*void sensor_reading() {
  long sensor_distance = sensor.readRangeSingleMillimeters();  /* sensor feedback from VL6180X
  unsigned long current_time = millis();
  unsigned long update_time = current_time - prev_time;
  if ( update_time > 5 ) {
    prev_time = current_time;              /* update prev_time
    pose.update( e0_count, e1_count);  /* update the kinematic
    xpose = pose.get_xpos();           /* local position
    ypose = pose.get_ypos();
    theta = pose.get_theta();
    //double distance;
  }
  }*/

void certainty_cal(int x_i, int y_j, int t) {
  //int t;                              // = Romi turning angle - sensor angle.
  int r = 100;                        // grid size, 100mm*100mm
  int k1, k2;                         // constant.
  int detect_range = 105;             // distance start to be taken into account.
  int safety_distance = 99;          // nearest obstacle can get.
  int certainty_map[3][3];
 
  int i, j;
  long a, b, d;
  int C_i_j;
 double F_rpx;                        /* Initialize force */
 double F_rpy;
  long sensor_distance = sensor.readRangeSingleMillimeters();   // distance calculation in each grid
    int real_distance=sensor_distance+85;
   pose.update( e0_count, e1_count);
       xpose = pose.get_xpos();           /* local position*/
    ypose = pose.get_ypos();
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
     if (i == 0) {
        k2 = 1;
      }
      else if (i == 1){
        k2 = 0;
      }
      else {
        k2 = -1;
      }
      if (j == 0) {
        k1 = -1;
      }
      else if (j == 1){
        k1 = 0;
      }
      else {
        k1 = 1;
      }
     if(sensor_distance<=500){
    a = (real_distance * cos(t) - k1 * r)/10;
      b = (real_distance* sin(t) - k2 * r)/10;
      d = sqrt(a*a + b*b);

       //Serial.println (a);
        //Serial.println (b);
           //Serial.println (d);
      // define certainty value, distance range (140,200)
    
      if ( d > detect_range) {
        C_i_j = 10;                            // tier 1: far away but detected.
        //Serial.println("go 1" );
      }
      else if (d < safety_distance) {
        C_i_j = 30;                            // tier 3: too close for crash.
        //Serial.println ("go 2");
      }
      else {
        C_i_j = 20;                            // tier 2: close.
        //Serial.println ("go s3");
      }
       
    }
   certainty_map[i][j] = C_i_j;
      //F_rpx = (F_cr * certainty_map[i][j]  / (d * d)) * (a/ d);
          //F_rpy = (F_cr * certainty_map[i][j]  / (d * d)) * (b / d);
         // Serial.println( C_i_j);
   F_rpx = (F_cr * C_i_j  / (d * d)) * (k1*r/ d);
          F_rpy = (F_cr *C_i_j  / (d * d)) * (k2*r / d);
       
 //Serial.println(  F_rpx);
      F_rp_x += F_rpx; //排斥力
      F_rp_y += F_rpy;
  }
  //Serial.println( F_cr);
  //Serial.println(  sensor_distance);
  //Serial.println(  x_i);
   //Serial.println(  y_j);
     
      //Serial.println(  a);
      //Serial.println( b);
       //Serial.println( d);
       // Serial.println(  F_rp_x);
        //Serial.println(  xpose);
 
  }
 // Serial.println(  F_rp_x);
   //return F_rp_x, F_rp_y;
   
    for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
 // Serial.println( certainty_map[i][j]);
  }}}
  /*if(sensor_distance<212){
        int x_i =  xpose + sensor_distance * cos(t);               // cordinates of deteced distance.
    int y_j = ypose + sensor_distance * sin(t);
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
     F_rpx = (F_cr * certainty_map[i][j]  / (sensor_distance*sensor_distance )) * ((x_i -  xpose) / sensor_distance);
     
     F_rpy = (F_cr * certainty_map[i][j]  / (sensor_distance ^2) ) * ((y_j -  ypose) / sensor_distance); //格子到障碍物的排斥
  }}
   //Serial.println( F_cr);
  Serial.println(  sensor_distance);
  //Serial.println(  x_i);
   //Serial.println(  y_j);
      Serial.println(  F_rpx);
        //Serial.println(  F_rp_x);
        //Serial.println(  xpose);
  return F_rp_x, F_rp_y;
}*/


void Force(float F_rp_x, float F_rp_y , int xpose, int ypose, float theta) {
    pose.update( e0_count, e1_count);
       xpose = pose.get_xpos();           /* local position*/
    ypose = pose.get_ypos();
    
  int i, j;
  //float theta1;
  float theta_to_turn;
 //int p=FinalX -  xpose;
    //int q=FinalX -  xpose;
    //int e=p*p;
   // int g=q*q;
//  for (i = 0; i < 3; i++) {
//    for (j = 0; j < 3; j++) {
//      F_rpx = (F_cr * certainty_map[i][j]  / sensor_distance ^ 2) * ((x_i -  xpose) / sensor_distance);
//      F_rpy = (F_cr * certainty_map[i][j]  / sensor_distance ^ 2) * ((y_j -  ypose) / sensor_distance); //格子到障碍物的排斥力
//      F_rp_x += F_rpx;
//      F_rp_x += F_rpy;
//    }
//  }
double d_t;                                                    // distance between target and robot position.
    //d_t = sqrt(e+ g);
     //d_t = sqrt(333.33);
      //d_t = sqrt(2*(p/100)*(q/100));
     float p=(FinalX -  xpose)/100;
      float q=(FinalY -  ypose)/100;
           d_t = sqrt(2*p*q);
 //d_t = sqrt(((FinalX -  xpose)/100)^2 + ((FinalY -  ypose))/100)^2);
    //d_t = sqrt(((FinalX -  xpose)/100)*((FinalX -  xpose)/100) + ((FinalY -  ypose)/100)*((FinalY -  ypose)/100)));
  F_at_x = F_ct * ((FinalX - xpose) / d_t); //吸引力
  F_at_y = F_ct * ((FinalY - ypose) / d_t);
  F_x = (-F_rp_x) + F_at_x;
  F_y = (-F_rp_y) + F_at_y;

  //Serial.println(  F_at_x);
 //Serial.println( F_x);
  //Serial.println( F_y);
  //Serial.println(  F_rp_x);
   //Serial.println( F_x);
 //Serial.println( d_t);
 //Serial.println( F_rp_y);
   theta1 = atan2(F_y, F_x);
    
   //Serial.println(theta1);

}

 void Change(float theta1) {
  //float theta1;
  double  d_t;
  long sensor_distance = sensor.readRangeSingleMillimeters();  /* sensor feedback from VL6180X*/
  pose.update( e0_count, e1_count);
  theta = pose.get_theta();
  //Serial.println (  xpose);
  //TurnAngle = theta1 -(theta+PI/2);
    TurnAngle = (PI/2-theta1) -theta;
  //Serial.println (  theta1);
  //Serial.println(  TurnAngle);
  speed = 20.0f;
  /* turning right*/
  if ((-0.1 < TurnAngle) && (TurnAngle < 0.1)) {
    leftMotor(speed);
    rightMotor(speed);                /* go straight*/
    //Serial.println ("go straight");
  }
  else if (TurnAngle < -0.1) //左转
  {
    leftMotor(-speed);
    rightMotor(speed);
    delay(100);
    leftMotor(power);
    rightMotor(power);
     // Serial.println ("go left");
  }
  else
  { leftMotor(speed); //右转
    rightMotor(-speed);
    delay(100);
    leftMotor(power);
    rightMotor(power);
     //Serial.println ("go right");
  }
}

void printdata()
{ unsigned long current_time = millis();
 unsigned long update_time = current_time - prev_time;
 if ( update_time > 500 ) {
    prev_time = current_time;  
    pose.update( e0_count, e1_count);  /* update the kinematics*/
    xpose = pose.get_xpos();           /* local position*/
    ypose = pose.get_ypos();
    Serial.println (  xpose);
    Serial.println (  ypose);
  
    }}
