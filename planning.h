#ifndef PLANNING_H
#define PLANNING_H

#include "car.h"
#include "map.h"
#include <math.h>
#include <string>
#include <vector>



using std::vector;

// 1m/s =mph/2.24
const int lanes_available = 3;
const double max_vel = 49.5; //max speed = 50mph --> 25m/s
const double max_var_vel = 0.224; // 5m/s2 --> 5*2.24 = 11.2 mph --> 11.2*50times/s = 5m/s
// T =0.02s

void planAction(const int &prev_size, const vector< vector<double> > &sensor_fusion, Car &egoCar, const double &end_path_s, CarController &goal ){

	if(prev_size > 0)
      {
        egoCar.s = end_path_s;
      }

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

vector<string> successor_states(Car car) {
  // Provides the possible next states given the current state for the FSM 
  //   discussed in the course, with the exception that lane changes happen 
  //   instantaneously, so LCL and LCR can only transition back to KL.
  vector<string> states;
  states.push_back("KL");
  string state = car.state;
  if(car.state.compare("KL") == 0) {
    states.push_back("PLCL");
    states.push_back("PLCR");
  } else if (car.state.compare("PLCL") == 0) {
    if (car.lane != lanes_available - 1) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  } else if (car.state.compare("PLCR") == 0) {
    if (car.lane != 0) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }
    
  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

CarController constant_speed_trajectory(const CarController &goal){
	return goal;
}

CarController keep_lane_trajectory(const CarController &goal,const vector< vector<double> > &sensor_fusion){
	return goal;
}
CarController lane_change_trajectory(const CarController &goal,const string &state, const vector< vector<double> > &sensor_fusion){
	return goal;
}
CarController prep_lane_change_trajectory(const CarController &goal,const string &state, const vector< vector<double> > &sensor_fusion){
	return goal;
}


CarController generate_trajectory(Car car, const string &state, 
                                             const vector< vector<double> > &sensor_fusion) {
  // Given a possible next state, generate the appropriate trajectory to realize
  //   the next state.
  CarController goal;
  if (state.compare("CS") == 0) {
    goal = constant_speed_trajectory(goal);
  } else if (state.compare("KL") == 0) {
    goal = keep_lane_trajectory(goal,sensor_fusion);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    goal = lane_change_trajectory(goal,state, sensor_fusion);
  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    goal = prep_lane_change_trajectory(goal,state, sensor_fusion);
  }
  return goal;
}

#endif