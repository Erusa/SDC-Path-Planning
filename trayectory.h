#ifndef TRAYECTORY_H
#define TRAYECTORY_H

#include "car.h"
#include "map.h"
#include <math.h>
#include <string>
#include <vector>
#include "spline.h"

using std::vector;

vector < vector<double> >  generateTrayectory(int lane, double ref_vel, Car car, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d, double prev_size, Map map ){

	// create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
    vector<double> ptsx;
    vector<double> ptsy;
    // reference x,y, yaw states 
    double ref_x = car.x;
    double ref_y = car.y;
    double ref_yaw = deg2rad(car.yaw);
    //if previous list is almost empty, use the car as starting reference
    if(prev_size<2)
    {
      //Use two points that make the path tangent to the car
      double prev_car_x =  car.x - cos(car.yaw);
      double prev_car_y = car.y - sin(car.yaw);

      ptsx.push_back(prev_car_x);
      ptsx.push_back(car.x);

      ptsy.push_back(prev_car_y);
      ptsy.push_back(car.y);
    }
    //use the previous paths and point as starting reference
    else
    {
      //redefine reference state as previous path and point
      ref_x = previous_path_x[prev_size - 1];
      ref_y = previous_path_y[prev_size - 1];

      double ref_x_prev = previous_path_x[prev_size - 2];
      double ref_y_prev = previous_path_y[prev_size - 2];
      ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

      //Use two points that make the path tangen to the previous path's end point
      ptsx.push_back(ref_x_prev);
      ptsx.push_back(ref_x);

      ptsy.push_back(ref_y_prev);
      ptsy.push_back(ref_y);
    }

    //In Frenet add evenly 30m spaced points ahead of the starting reference
    vector<double> next_wp0 = getXY(car.s+30, (2+4*lane), map.waypoints_s, map.waypoints_x,          map.waypoints_y);
    vector<double> next_wp1 = getXY(car.s+60, (2+4*lane), map.waypoints_s, map.waypoints_x,          map.waypoints_y);
    vector<double> next_wp2 = getXY(car.s+90, (2+4*lane), map.waypoints_s, map.waypoints_x,          map.waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i = 0; i < ptsx.size(); ++i) {

      //shift car reference angle to 0 degree
      double shift_x = ptsx[i]- ref_x;
      double shift_y = ptsy[i]- ref_y;

      ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
      ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
    }

    //create a Spline
    tk::spline s;

    //set(x,y) points to the spline
    s.set_points(ptsx, ptsy);

    //Define the actual(x,y) points we will use for the planner
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    //Start with all of the previous path points from last time
    for(int i=0; i<previous_path_x.size(); ++i){
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
    }

    //Calculate how to brak up splice points so that we traver at our desired reference velocity
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));

    double x_add_on = 0.0;

    //fill up the rest of our path planner after filling it with previos points, here we will always output 50 points
    //Elsa: here change of acceleration can also be written. Here woud be better
    for (int i =1; i<= 50 - previous_path_x.size(); ++i){
      double N = target_dist
        /(0.02*ref_vel/2.24);
      double x_point = x_add_on + target_x/N;
      double y_point = s(x_point);

      x_add_on = x_point;
      double x_ref = x_point;
      double y_ref = y_point;

      //rotate back to normal after rotating it earlier
      x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
      y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

      x_point += ref_x;
      y_point += ref_y;

      next_x_vals.push_back(x_point);
      next_y_vals.push_back(y_point);
    }
  
  vector < vector<double> > next_points;
  next_points.push_back(next_x_vals);
  next_points.push_back(next_y_vals);
  
  return next_points;
}


#endif  // TRAYECTORY_H