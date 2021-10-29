#ifndef COST_H
#define COST_H

#include "car.h"
#include "map.h"
#include <math.h>
#include <string>
#include <vector>
#include "cost.h"

using std::vector;

const float COLLISION = 500; //1000
const float SPEED_LIMIT = 100;
const float SPEED = 10;
const float CHANGE = 10; //1
//const float REACH_GOAL = pow(10, 6);
//const float EFFICIENCY = pow(10, 5);

extern double max_vel;
extern double max_var_vel; 

float collision_cost(const CarController goal, const vector< vector<double> > &sensor_fusion, const Car &egoCar, const int &prev_size){
 //find ref_v to use
  float cost = 0;
    for(int i=0; i<sensor_fusion.size(); ++i){
      //car is in my lane
      float d = sensor_fusion[i][6];
      if(d < (2+4*goal.lane+2) && d > (2+4*goal.lane-2))
      {
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx*vx + vy*vy);
        double check_car_s = sensor_fusion[i][5];

        // predict car position
        check_car_s += (double)prev_size* 0.02*check_speed; //if using previous points can project s value out
        //check s values greater than mine and s gap
        if((check_car_s > egoCar.s) && (check_car_s - egoCar.s < 60))
        { 
          float new_cost = COLLISION*pow(30 - (check_car_s - egoCar.s),2);
          cost = new_cost > cost ? new_cost : cost;
         }                                
      }
    }
	return cost;
}

float speed_cost(const CarController goal){
  float cost = 0;
  if (goal.speed > max_vel){
  	cost = SPEED_LIMIT * (goal.speed- max_vel);
  }
  else
  {
    cost = SPEED * (max_vel - goal.speed);
  }
	return cost;
}

float change_cost(const CarController goal,  const Car &egoCar){
	float cost = goal.lane == egoCar.lane ?  0: CHANGE;
	return cost;
}
                                         
/*float acc_limit_cost(const CarController goal){
  float cost = 0;
  if (goal.speed > max_vel){
  	cost = ACC_LIMIT * (goal.speed- max_vel)
  }
	return cost;
}*/

float outsideroad_cost(const CarController goal, const Car &egoCar){
	float cost =  goal.lane;
}

float calculate_cost(const CarController goal, const vector< vector<double> > &sensor_fusion, const Car &car, const int &prev_size){
  float cost = collision_cost(goal,sensor_fusion, car, prev_size);
  cost += speed_cost(goal);
  cost += change_cost(goal, car);
  return cost;
}
#endif