#ifndef PLANNING_H
#define PLANNING_H

#include "car.h"
#include "map.h"
#include <math.h>
#include <string>
#include <vector>

using std::vector;

void planAction(int prev_size, vector< vector<double> > sensor_fusion, Car &egoCar, double end_path_s, double &ref_vel, int& lane ){

	if(prev_size > 0)
      {
        egoCar.s = end_path_s;
      }

    bool too_close = false;

    //find ref_v to use
    for(int i=0; i<sensor_fusion.size(); ++i){
      //car is in my lane
      float d = sensor_fusion[i][6];
      if(d < (2+4*lane+2) && d > (2+4*lane-2))
      {
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx*vx + vy*vy);
        double check_car_s = sensor_fusion[i][5];

        check_car_s += (double)prev_size* 0.02*check_speed; //if using previous points can project s value out
        //check s values greater than mine and s gap
        if((check_car_s > egoCar.s) && (check_car_s - egoCar.s < 30))
        {
          //do some logic here, lower reference velocity so we dont crash into the car infrot of us
          // also flag to try to change lanes
          //ref_vel = 29.5; //mph
          too_close=true;
          if (lane > 0){
            lane = 0;
          }
        }
      }
    }

    if (too_close)
    {
      ref_vel -= 0.224;
      if (ref_vel<0) {ref_vel=0;}
    }
    else if (ref_vel <49.5){
      ref_vel += 0.224;
    }
  
}





#endif