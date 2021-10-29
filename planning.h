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

// 1m/s =mph/2.24
int lanes_available = 3;
double max_vel = 49.5; //max speed = 50mph --> 25m/s
double max_var_vel = 0.224; // 5m/s2 --> 5*2.24 = 11.2 mph --> 11.2*50times/s = 5m/s
// T =0.02s

void planAction(const int &prev_size, const vector< vector<double> > &sensor_fusion, Car &egoCar, CarController &goal ){

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
        if((check_car_s > egoCar.s) && (check_car_s - egoCar.s < 30))
        {
          //do some logic here, lower reference velocity so we dont crash into the car infrot of us
          // also flag to try to change lanes
          //ref_vel = 29.5; //mph
          too_close=true;
          if (goal.lane > 0){
            goal.lane = 0;
          }
        }
      }
    }

    if (too_close)
    {
      goal.speed -= max_var_vel;
      if (goal.speed<0) {goal.speed=0;}
    }
    else if (goal.speed <max_vel){
      goal.speed += max_var_vel;
    }
  
}

vector<string> successor_states(Car &car) {
  // Provides the possible next states given the current state for the FSM 
  //   discussed in the course, with the exception that lane changes happen 
  //   instantaneously, so LCL and LCR can only transition back to KL.
  vector<string> states;
  states.push_back("KL");
  string state = car.state;
  if(car.state.compare("KL") == 0) {
    
    if (car.lane != lanes_available - 1) {
      states.push_back("LCR"); 
    	cout<< "add state LCR" <<endl;
    }
    	
    if (car.lane != 0) {
      //states.push_back("PLCR");
      states.push_back("LCL");  
    	cout<< "add state LCL" <<endl;
    }
  }
    
  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}


CarController constant_speed_trajectory(const CarController &old_goal){
  CarController goal = old_goal;
  if (old_goal.speed<0) {goal.speed=0;}
    
  else if (old_goal.speed <max_vel){
    goal.speed += max_var_vel;}
      
	return goal;
}

CarController keep_lane_trajectory(const CarController &old_goal, const Car &egoCar, const vector< vector<double> > &sensor_fusion, const int &prev_size){
  CarController goal = old_goal;

    bool too_close = false;

    //find ref_v to use
    for(int i=0; i<sensor_fusion.size(); ++i){
      //car is in my lane
      float d = sensor_fusion[i][6];
      if(d < (2+4*old_goal.lane+2) && d > (2+4*old_goal.lane-2))
      {
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx*vx + vy*vy);
        double check_car_s = sensor_fusion[i][5];

        // predict car position
        check_car_s += (double)prev_size* 0.02*check_speed; //if using previous points can project s value out
        //check s values greater than mine and s gap
        if((check_car_s > egoCar.s) && (check_car_s - egoCar.s < 30))
        {
          //do some logic here, lower reference velocity so we dont crash into the car infrot of us
          too_close=true;
        }
      }
    }

    if (too_close)
    {
      goal.speed -= max_var_vel;
      if (old_goal.speed<0) {goal.speed=0;}
    }
  
    else if (old_goal.speed <max_vel){
      goal.speed += max_var_vel;
    }
  
	return goal;
}
CarController lane_change_trajectory_left(const CarController &old_goal, const Car &car, const vector< vector<double> > &sensor_fusion){
  CarController goal=old_goal;
  goal.lane = (car.lane ==0 ? 99 : (car.lane-1)); 
	return goal;
}
             
CarController lane_change_trajectory_right(const CarController &old_goal, const Car &car,  const vector< vector<double> > &sensor_fusion){
  CarController goal = old_goal;
  goal.lane = (car.lane ==(lanes_available-1) ? 99 : (car.lane+1)); 
  return goal;
}
/*CarController prep_lane_change_trajectory(const CarController &goal,const string &state, const vector< vector<double> > &sensor_fusion){
	return goal;
}*/

CarController generate_goal(Car& car, const CarController &old_goal, const string &state, 
                                             const vector< vector<double> > &sensor_fusion, const int &prev_size) {
  // Given a possible next state, generate the appropriate trajectory to realize
  //   the next state.
  CarController goal;
  if (state.compare("CS") == 0) {
    goal = constant_speed_trajectory(old_goal);
  } 
  else if (state.compare("KL") == 0) {
    cout<< "computing goal when KL" <<endl;
    goal = keep_lane_trajectory(old_goal, car, sensor_fusion, prev_size);
  } 
  else if (state.compare("LCL") == 0) {
    cout<< "computing goal when LCL" <<endl;
    goal = lane_change_trajectory_left(old_goal, car, sensor_fusion);
  } 
  else if (state.compare("LCR") == 0) {
    goal = lane_change_trajectory_right(old_goal, car, sensor_fusion);
    cout<< "computing goal when LCR" <<endl;
  }
  //car.lane = goal.lane;
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
  //car.lane = final_goals[best_idx].lane;

  /**
   * TODO: Change return value here:
   */
  return final_goals[best_idx];
}



#endif