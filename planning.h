#ifndef PLANNING_H
#define PLANNING_H

#include "car.h"
#include "map.h"
#include <math.h>
#include <string>
#include <vector>
#include "cost.h"
#include <iostream>


using std::cout;
using std::endl;
using std::vector;

// 1m/s =mph/2.24 // T =0.02s
int lanes_available = 3;
double max_vel = 22; //max speed = 50mph --> 22.3m/s
double max_var_vel = 0.1;// max acc = 10 m/s2 --> each 0.02s will be 0.2



vector<string> successor_states(Car &car) {
  // Provides the possible next states given the current state of the car
  vector<string> states;
  states.push_back("KL");
  string state = car.state;
  if(car.state.compare("KL") == 0) {
    
    if (car.lane != lanes_available - 1) {
      states.push_back("LCR"); 
    	cout<< "add state LCR" <<endl;
    }
    	
    if (car.lane != 0) {
      states.push_back("LCL");  
    	cout<< "add state LCL" <<endl;
    }
  }
  else if (car.state.compare("CS") != 0)
   {
   		states.push_back(car.state); 
   }
  return states;
}


CarController constant_speed_trajectory(const CarController &old_goal){
  CarController goal = old_goal;
  if (old_goal.speed<0) {goal.speed=0;}
    
  else if (old_goal.speed <max_vel){
    goal.speed += max_var_vel;}
      
	return goal;
}

double change_speed(const CarController &goal, const Car &egoCar, const vector< vector<double> > &sensor_fusion, const int &prev_size){
	//predict ego car position
	double car_s = egoCar.s + prev_size*0.02*goal.speed - 0.5*prev_size*0.02*0.02*prev_size*max_var_vel;
  
    bool too_close = false;

    //find ref_v to use
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
        if((check_car_s > car_s) && (check_car_s - car_s < 25))
        {
          //do some logic here, lower reference velocity so we dont crash into the car infrot of us
          too_close=true;
        }
      }
    }
   /*Posible-Upgrade:     speed = car.speed;
   however car.speed display sometimes errors from frenet conversion singularity. */
   double speed = goal.speed;
  
    if (too_close)
    {
      speed -= max_var_vel;
      if (speed<0) {speed=0;}
    }
  
    else if (speed <max_vel){
      speed += max_var_vel;
    }
  
	return speed;
}

CarController keep_lane_trajectory(const CarController &old_goal, const Car &car, const vector< vector<double> > &sensor_fusion, const int &prev_size){
  
  CarController goal=old_goal;
  
  goal.lane = car.lane;
  goal.speed = change_speed(goal, car, sensor_fusion, prev_size);
  return goal;
}

CarController lane_change_trajectory_left(const CarController &old_goal, const Car &car, const vector< vector<double> > &sensor_fusion, const int &prev_size){
  
  CarController goal=old_goal;
  
  goal.lane = (car.lane ==0 ? 99 : (car.lane-1)); 
  goal.speed = change_speed(goal, car, sensor_fusion, prev_size);
  return goal;
}
             
CarController lane_change_trajectory_right(const CarController &old_goal, const Car &car,  const vector< vector<double> > &sensor_fusion, const int &prev_size){
  
  CarController goal = old_goal;
  
  goal.lane = (car.lane ==(lanes_available-1) ? 99 : (car.lane+1)); 
  goal.speed = change_speed(goal, car, sensor_fusion, prev_size);
  return goal;
}

CarController generate_goal(Car& car, const CarController &old_goal, const string &state, 
                                             const vector< vector<double> > &sensor_fusion, const int &prev_size) {
  // Given a possible next state, generate the appropriate goal for trajectory generation
  CarController goal;
  if (state.compare("CS") == 0) {
    goal = constant_speed_trajectory(old_goal);
  } 
  else if (state.compare("KL") == 0) {
    goal = keep_lane_trajectory(old_goal, car, sensor_fusion, prev_size);
  } 
  else if (state.compare("LCL") == 0) {
    goal = lane_change_trajectory_left(old_goal, car, sensor_fusion, prev_size);
  } 
  else if (state.compare("LCR") == 0) {
    goal = lane_change_trajectory_right(old_goal, car, sensor_fusion, prev_size);
  }
  return goal;
}
             
CarController choose_next_state(Car &car, const vector< vector<double> > &sensor_fusion, CarController &old_goal, const int &prev_size) {

  vector<string> states = successor_states(car);
  float cost;
  vector<float> costs;
  vector<CarController> final_goals;
  vector<string> valid_states;
	
  CarController  goal = old_goal;
  for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
    goal = generate_goal(car, old_goal, *it, sensor_fusion, prev_size);
       
    if (goal.lane != 99) {
      cost = calculate_cost(goal, sensor_fusion, car, prev_size);
      costs.push_back(cost);
      final_goals.push_back(goal);
      valid_states.push_back(*it);
      cout<<"cost from "<<*it<<" is "<<cost<<endl;
    }
  }

  vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);
  
  car.state = valid_states[best_idx];

  cout << "speed: " << final_goals[best_idx].speed << ", speed from car: " << car.speed <<endl;
  return final_goals[best_idx];
}


#endif