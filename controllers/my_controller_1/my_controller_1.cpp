#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/LightSensor.hpp>
#include <iostream>
#include <ctime>
#include <stdlib.h>

#define TIME_STEP 50

#define MAX_SPEED 6.28

using namespace webots;


int main(int argc, char **argv) 
{
  Robot *robot = new Robot();
  Motor *wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  
  for (int i = 0; i < 4; i++) 
  {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  
  LightSensor *ls[4];
  char lsNames[4][26] = {"light_sensor_front", "light_sensor_back", "light_sensor_left", "light_sensor_right"};
  
  for (int i = 0; i < 4; i++) 
  {
    ls[i] = robot->getLightSensor(lsNames[i]);
    ls[i]->enable(TIME_STEP/2);
  }
  srand(time(0));
  double random_err_arr[4], sensors_with_random[4], motor_err[4];
  double sum_err = 0.0, err = 0.0, last_err = 0.0, PID_err = 0.0;
  for(int i = 0; i < 4; i++)
    motor_err[i] = (double)(rand())/RAND_MAX * 0.5 - 0.25;
  
  // p = 0.06
  double K_p = 0.03, K_i = 0, K_d = 0;
  // double T = 1.0 / 5.0;
  // K_p = K_p * 0.6;
  // K_i = 2.0 * K_p / T  * (TIME_STEP / 1000.0);
  // K_d = K_p * T / 8.0  / (TIME_STEP / 1000.0);
  // K_i /= 7;
  // K_d *= 1.25;
  
  double T = 9.25 / (76.0 / 2.0);
  K_p = K_p * 0.6;
  K_i = 2.0 * K_p / T  * (TIME_STEP / 1000.0);
  K_d = K_p * T / 8.0  / (TIME_STEP / 1000.0);
  K_i /= 15;

  
  
  std::cout << K_p << ' ' <<K_i << ' ' << K_d << '\n';
 
  // int counter = 0;
  
  while (robot->step(TIME_STEP) != -1)
  {
   
  
      for (int i = 0; i < 4; i++) 
      {
        random_err_arr[i] = (double)(rand())/RAND_MAX * 100 - 50;
        sensors_with_random[i] = ls[i]->getValue() + random_err_arr[i];
      }
      
      if (sensors_with_random[0] < sensors_with_random[1])
      {
        wheels[0]->setVelocity(-(5.0 + motor_err[0]));
        wheels[2]->setVelocity(-(5.0 + motor_err[2]));
        wheels[1]->setVelocity((5.0 + motor_err[1]));
        wheels[3]->setVelocity((5.0 + motor_err[3]));
        continue;
      }
      err = sensors_with_random[2] - sensors_with_random[3];
      sum_err += err;
      PID_err = K_p * err + K_i * sum_err + K_d * (last_err - err);
      wheels[0]->setVelocity(5.0 - PID_err + motor_err[0]);
      wheels[2]->setVelocity(5.0 - PID_err + motor_err[2]);
      wheels[1]->setVelocity(5.0 + PID_err + motor_err[1]);
      wheels[3]->setVelocity(5.0 + PID_err + motor_err[3]);
      
      // if(last_err * err < 0)
        // counter++;
      // std::cout << counter << '\n';
      // last_err = err;

  } 
  delete robot;
  return 0;
}