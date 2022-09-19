/*
 * File: PID_wall_follower.c
 * Description: Controller for wall robot using PID
 * Author: Alim Satria Fi'i Wijaya Kusuma
 * Modifications:
 */

#include <stdio.h>
#include <webots/robot.h>

#include <webots/distance_sensor.h>
#include <webots/device.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>

#define TIME_STEP 1
#define SAMPLING 100
#define BASE_SPEED 2.64
#define DISTANCE_SENSORS_NUMBER 8
#define LIGHT_SENSORS_NUMBER 8
#define GROUND_SENSORS_NUMBER 3
#define MOTORS_NUMBER 2

static const char *distance_sensors_names[DISTANCE_SENSORS_NUMBER] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
static const char *light_sensors_names[LIGHT_SENSORS_NUMBER] = {"ls0", "ls1", "ls2", "ls3", "ls4", "ls5", "ls6", "ls7"};
static const char *ground_sensors_names[GROUND_SENSORS_NUMBER] = {"gs0","gs1", "gs2"};
static const char *motors_names[MOTORS_NUMBER] = {"left wheel motor","right wheel motor"};
//--------------------------------------------------------------------// PID VAR
double error = 0,lastError = 0, totalError, deltaError;
double setPoint = 130, controlSignal;
double KP_Control, KI_Control, KD_Control, KP = 200, KI = 0.01, KD = 1;
double motorControl = 0;
//--------------------------------------------------------------------// MEMORY ARRAY
char mazeMemory[64];
char condition;
//--------------------------------------------------------------------//

double constrainControl(double inputValue,int minValue, int maxValue);
double map(double input, double inputMin, double inputMax, double outputMin, double outputMax);
double readSensor(int x);
double readLightSensor(int x);
double readGroundSensor(int x);
double averageSensor(void);
double averageGroundSensor(void);
void averageHeadingSensor(void);

void printSensor(void);

double PID();

int main(int argc, char **argv) {

  //init_robot
  wb_robot_init();

  WbDeviceTag motor_left  = wb_robot_get_device(motors_names[0]);
  WbDeviceTag motor_right = wb_robot_get_device(motors_names[1]);


  while (wb_robot_step(TIME_STEP) != -1) {
  
    if(averageGroundSensor() > 350){
  
      // printSensor();
      averageHeadingSensor();
      printf("[PID] control : %f\n", PID(averageSensor()));
      
      motorControl = PID(averageSensor());
  
      wb_motor_set_position(motor_left,INFINITY);
      wb_motor_set_position(motor_right,INFINITY);
  
      wb_motor_set_velocity(motor_left, BASE_SPEED - motorControl); // set kecepatan motor kiri
      wb_motor_set_velocity(motor_right, BASE_SPEED + motorControl); // set kecepatan motor kanan
  
    }
    else{
    
      wb_motor_set_position(motor_left,0);
      wb_motor_set_position(motor_right,0);
  
      wb_motor_set_velocity(motor_left,0); // set kecepatan motor kiri
      wb_motor_set_velocity(motor_right,0); // set kecepatan motor kanan
      
    }

  };

  wb_robot_cleanup();

  return 0;
}

double constrain(double inputValue,int minValue, int maxValue){

  if (inputValue > maxValue)
    inputValue = maxValue;
  else if (inputValue < minValue)
    inputValue = minValue;

  return inputValue;

}

double PID(double distance){

  error = setPoint - distance;
  
  // error = constrain(error,-40,40);
  
  totalError += error;

  totalError = constrain(totalError, -50, 50);
  // totalError = constrain(totalError, -35, 35);

  deltaError = error - lastError;

  KP_Control = KP * error;
  KI_Control = KI * totalError;
  KD_Control = KD * deltaError;

  controlSignal = KP_Control + KI_Control + KD_Control;

  controlSignal = constrain(controlSignal, -10000, 10000);
  controlSignal = map(controlSignal,-10000,10000,-2.64,2.64);

  lastError = error;

  return controlSignal;

}

double readSensor(int x){

  WbDeviceTag distance_sensors[x];
  distance_sensors[x] = wb_robot_get_device(distance_sensors_names[x]);
  wb_distance_sensor_enable(distance_sensors[x], TIME_STEP);

  return wb_distance_sensor_get_value(distance_sensors[x]);

}

double readLightSensor(int x){

  WbDeviceTag light_sensors[x];
  light_sensors[x] = wb_robot_get_device(light_sensors_names[x]);
  wb_light_sensor_enable(light_sensors[x], TIME_STEP);

  return wb_light_sensor_get_value(light_sensors[x]);

}

double readGroundSensor(int x){

  WbDeviceTag ground_sensors[x];
  ground_sensors[x] = wb_robot_get_device(ground_sensors_names[x]);
  wb_distance_sensor_enable(ground_sensors[x], TIME_STEP);
  
  return wb_distance_sensor_get_value(ground_sensors[x]);
}

double averageSensor(void){

  double averageVal = 0;

  averageVal = (readSensor(6)*0.4 + readSensor(0)*0.4 + readSensor(7)*0.2) / (0.4*1 + 0.4*1 + 0.2*1 );

  return averageVal;

}

double averageGroundSensor(void){
  
  double averageVal;
  
  averageVal = (readGroundSensor(0) + readGroundSensor(1) + readGroundSensor(2)) / 3;
  
  return averageVal;

}

double map(double input, double inputMin, double inputMax, double outputMin, double outputMax){

  return (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;

}

void printSensor(void){

  for(int x = 0; x < DISTANCE_SENSORS_NUMBER; x++){
    printf("[sensor] distance [%d] : %f\n", x, readSensor(x));
  }
  for(int x = 0; x < LIGHT_SENSORS_NUMBER; x++){
    printf("[sensor] light [%d] : %f\n", x, readLightSensor(x));
  }
  for(int x = 0; x < GROUND_SENSORS_NUMBER; x++){
    printf("[sensor] ground [%d] : %f\n", x, readGroundSensor(x));
  }
  
  printf("[sensor] average distance : %f\n", averageSensor());
  printf("[sensor] average ground : %f\n", averageGroundSensor());
  
  printf("\n");

}

void averageHeadingSensor(void){
  
  double averageFront = 0, averageRight = 0, averageLeft = 0;
  
  for(int x = 0; x < SAMPLING; x++)
    averageFront += ((readSensor(0) + readSensor(7)) / 2);
  
  averageFront = averageFront / 100;
  
  for(int x = 0; x < SAMPLING; x++)
    averageRight += (readSensor(2));
  
  averageFront = averageRight / 100;
  
  for(int x = 0; x < SAMPLING; x++)
    averageLeft += readSensor(5);
  
  averageLeft = averageLeft / 100;
  
  printf("[sensor] average front : %f\n",averageFront);
  printf("[sensor] average left : %f\n",averageLeft);
  printf("[sensor] average right : %f\n",averageRight);
  
}