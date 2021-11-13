// ================================================================
// ===               INCLUSION DE LIBRERIAS                     ===
// ================================================================
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif
#include <Servo.h>
#include <ros.h>
#include <Braccio.h>
#include <std_msgs/Float64.h>
//#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_ver;
Servo wrist_rot;
Servo gripper;
// ================================================================
// ===        INICIALIZACION DE ANGULOS PARA MOTORES            ===
// ================================================================

float baseM1 = 0;
float shoulderM2 = 45;
float elbowM3 = 180;
float wrist_verM4 = 0;
float wrist_rotM5 =180;
float gripperM6 =90; 
#define pi 3.1415

// ================================================================
// ===        Espacio de construccion de funciones              ===
// ================================================================

void M1( const std_msgs::Float64& angulo){
  baseM1=(float)angulo.data;
  baseM1=baseM1/pi* 180;
}
void M2( const std_msgs::Float64& angulo){
  shoulderM2=(float)angulo.data;
  shoulderM2=shoulderM2/pi*180;
}
void M3( const std_msgs::Float64& angulo){
  elbowM3=(float)angulo.data;
  elbowM3=elbowM3/pi*180;
}
void M4( const std_msgs::Float64& angulo){
  wrist_verM4 =(float)angulo.data;
  wrist_verM4=wrist_verM4/pi * 180;
}
void M5( const std_msgs::Float64& angulo){
  wrist_rotM5 =(float)angulo.data;
  wrist_rotM5=wrist_rotM5/pi*180;
}
void M6( const std_msgs::Float64& angulo){
  gripperM6 =  (float)angulo.data;
  gripperM6=map(gripperM6,0,pi,10,90);
}
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Float64> sub_1("/robot/joint1_position_controller/command",M1);
ros::Subscriber<std_msgs::Float64> sub_2("/robot/joint2_position_controller/command",M2);
ros::Subscriber<std_msgs::Float64> sub_3("/robot/joint3_position_controller/command",M3);
ros::Subscriber<std_msgs::Float64> sub_4("/robot/joint4_position_controller/command",M4);
ros::Subscriber<std_msgs::Float64> sub_5("/robot/joint5_position_controller/command",M5);
ros::Subscriber<std_msgs::Float64> sub_6("/robot/gripper/command",M6);

void setup(){
  nh.initNode();
  nh.subscribe(sub_1);
  nh.subscribe(sub_2);
  nh.subscribe(sub_3);
  nh.subscribe(sub_4);
  nh.subscribe(sub_5);
  nh.subscribe(sub_6);
  Braccio.begin();
}
void loop(){
  Braccio.ServoMovement(20,baseM1,shoulderM2,elbowM3,wrist_verM4,wrist_rotM5,gripperM6);
  nh.spinOnce();
  delay(1);
}
