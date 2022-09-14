 
/*
 * File: PID_wall_follower.c
 * Description: Controller for wall robot using PID
 * Author: Alim Satria Fi'i Wijaya Kusuma
 * Modifications:
 */

#include <stdio.h>
#include <webots/robot.h>

#include <webots/distance_sensor.h>
#include <webots/motor.h>

#define TIME_STEP 1
#define BASE_SPEED 2.64
#define DISTANCE_SENSORS_NUMBER 8
#define MOTORS_NUMBER 2

static const char *distance_sensors_names[DISTANCE_SENSORS_NUMBER] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
static const char *motors_names[MOTORS_NUMBER] = {"left wheel motor","right wheel motor"};
//--------------------------------------------------------------------// PID VAR
double error = 0,lastError = 0, totalError, deltaError;
double setPoint = 99.50, controlSignal;
double KP_Control, KI_Control, KD_Control, KP = 95, KI = 0.00001, KD = 1;
//--------------------------------------------------------------------//

double constrainControl(double inputValue,int minValue, int maxValue);
double map(double input, double inputMin, double inputMax, double outputMin, double outputMax);
double readSensor(int x);
double averageSensor(void);

void printSensor(void);

double PID();

int main(int argc, char **argv) {

  //init_robot
  wb_robot_init();

  WbDeviceTag motor_left  = wb_robot_get_device(motors_names[0]);
  WbDeviceTag motor_right = wb_robot_get_device(motors_names[1]);


  while (wb_robot_step(TIME_STEP) != -1) {

  averageSensor();
  printSensor();
  printf("kendali PID : %f\n", PID(averageSensor()));
  
  wb_motor_set_position(motor_left,INFINITY);
  wb_motor_set_position(motor_right,INFINITY);
  
  wb_motor_set_velocity(motor_left, BASE_SPEED - PID(averageSensor())); // set kecepatan motor kiri
  wb_motor_set_velocity(motor_right, BASE_SPEED + PID(averageSensor())); // set kecepatan motor kanan

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

double averageSensor(void){

  double averageVal = 0;

  averageVal = (readSensor(6)*0.4 + readSensor(0)*0.4 + readSensor(7)*0.2) / (0.4*1 + 0.4*1 + 0.2*1 );

  return averageVal;

}

double map(double input, double inputMin, double inputMax, double outputMin, double outputMax){

  return (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;

}

void printSensor(void){

  for(int x = 0; x < DISTANCE_SENSORS_NUMBER; x++){
    printf("Sensor [%d] : %f\n", x, readSensor(x));
  }
  printf("Average value : %f\n", averageSensor());
  printf("\n");

}