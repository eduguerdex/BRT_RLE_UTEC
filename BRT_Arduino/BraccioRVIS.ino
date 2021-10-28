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
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
/*  USO DIRECTO
//rostopic pub joint_array std_msgs/Float32MultiArray '{data: [180,90,90,90,90,70]}' --once
// rosrun rosserial_python serial_node.py /dev/ttyACM0
*/
// ================================================================
// ===         DECLARACION DE SERVO PARA BRACCIO                ===
// ================================================================
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_ver;
Servo wrist_rot;
Servo gripper;

// ================================================================
// ===        INICIALIZACION DE ANGULOS PARA MOTORES            ===
// ================================================================
float baseM1 = 90;
float shoulderM2 = 90;
float elbowM3 = 20;
float wrist_verM4 = 0;
float wrist_rotM5 = 0;
float gripperM6 =45; 
#define pi 3.1415

// ================================================================
// ===        Espacio de construccion de funciones              ===
// ================================================================

//sudo chmod 777 /dev/ttyACM0
void Movimiento( const std_msgs::Float32MultiArray&angulo){
  baseM1 =     (float)angulo.data[0];
  shoulderM2 = (float)angulo.data[1];
  elbowM3 =    (float)angulo.data[2];
  wrist_verM4 =(float)angulo.data[3];
  wrist_rotM5 =(float)angulo.data[4];
  gripperM6 =  (float)angulo.data[5];
  gripperM6=map(gripperM6,60,100,10,90);
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Float32MultiArray> sub("joint_multiarray", Movimiento);
// ================================================================
// ===                      SETUP INICIAL                      ===
// ==============================================================
void setup()
{
  Braccio.begin();
  nh.initNode();
  nh.subscribe(sub);

}
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop()
{
  Braccio.ServoMovement(20,baseM1,shoulderM2,elbowM3,wrist_verM4,wrist_rotM5,gripperM6);
  nh.spinOnce();
  delay(1);
}
