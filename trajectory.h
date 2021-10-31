#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "car.h"
#include "map.h"
#include <math.h>
#include <iostream>
#include <vector>
#include "spline.h"
#include "points.h"

using std::vector;
using std::cout;
using std::endl;

void choosePoints(const int &lane, SimpleCar &refCar, const Points &previous_path, const double &prev_size, const Map &map, Points& pts){
  
  //if previous list is almost empty, use the car as starting reference
    if(prev_size<2)
    {
      //Use two points that make the path tangent to the car
      double prev_car_x = refCar.x - cos(refCar.yaw);
      double prev_car_y = refCar.y - sin(refCar.yaw);

      pts.x.push_back(prev_car_x);
      pts.x.push_back(refCar.x);

      pts.y.push_back(prev_car_y);
      pts.y.push_back(refCar.y);
    }
    //use the previous paths and point as starting reference
    else
    {
      //redefine reference state as previous path and point
      refCar.x = previous_path.x[prev_size - 1];
      refCar.y = previous_path.y[prev_size - 1];

      double ref_x_prev = previous_path.x[prev_size - 2];
      double ref_y_prev = previous_path.y[prev_size - 2];
      refCar.yaw = atan2(refCar.y - ref_y_prev, refCar.x - ref_x_prev);

      //Use two points that make the path tangen to the previous path's end point
      pts.x.push_back(ref_x_prev);
      pts.x.push_back(refCar.x);

      pts.y.push_back(ref_y_prev);
      pts.y.push_back(refCar.y);
    }

  	//getLastPoint(car, lane, map, ptsx, ptsy);
    //In Frenet add evenly 30m spaced points ahead of the starting reference
    vector<double> next_wp0 = getXY(refCar.s+30, (2+4*lane), map.waypoints_s, map.waypoints_x,          map.waypoints_y);
    vector<double> next_wp1 = getXY(refCar.s+60, (2+4*lane), map.waypoints_s, map.waypoints_x,          map.waypoints_y);
    vector<double> next_wp2 = getXY(refCar.s+90, (2+4*lane), map.waypoints_s, map.waypoints_x,          map.waypoints_y);

    pts.x.push_back(next_wp0[0]);
    pts.x.push_back(next_wp1[0]);
    pts.x.push_back(next_wp2[0]);
    //cout<<"p4:"<< next_wp1[0]<<endl;
  	//cout<<"p5:"<< next_wp2[0]<<endl;

    pts.y.push_back(next_wp0[1]);
    pts.y.push_back(next_wp1[1]);
    pts.y.push_back(next_wp2[1]);
}

void shiftPoints2CarReference(Points &pts, const SimpleCar &ref){
    for (int i = 0; i < pts.x.size(); ++i) {

      //shift car reference angle to 0 degree
      double shift_x = pts.x[i]- ref.x;
      double shift_y = pts.y[i]- ref.y;

      pts.x[i] = (shift_x*cos(0-ref.yaw)-shift_y*sin(0-ref.yaw));
      pts.y[i] = (shift_x*sin(0-ref.yaw)+shift_y*cos(0-ref.yaw));
      //cout<<"px:"<< pts.x[i]<<endl;
    }
}

void choosePointsAccordingSpeed_shift2MapReference_getTrajectory(const double ref_vel, const tk::spline& s, const SimpleCar &ref, const Points &previous_path, Points& next_vals){
	//Calculate how to break up splice points so that we traver at our desired reference velocity
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));

    double x_add_on = 0.0;

    //fill up the rest of our path planner after filling it with previos points, here we will always output 50 points
    //Approach: here change of acceleration can also be written. Here woud be better
    for (int i =1; i<= 50 - previous_path.x.size(); ++i){
      //double N = target_dist/(0.02*ref_vel/2.24);
      double N = target_dist/(0.02*ref_vel);
      double x_point = x_add_on + target_x/N;
      double y_point = s(x_point);
      //cout<<"ps:"<< y_point <<endl;

      x_add_on = x_point;
      double x_ref = x_point;
      double y_ref = y_point;

      //rotate back to normal after rotating it earlier
      x_point = x_ref*cos(ref.yaw) - y_ref*sin(ref.yaw);
      y_point = x_ref*sin(ref.yaw) + y_ref*cos(ref.yaw);

      x_point += ref.x;
      y_point += ref.y;

      next_vals.x.push_back(x_point);
      next_vals.y.push_back(y_point);
    }
  
}

Points generateTrajectory(const CarController &goal, const Car &car, const Points &previous_path, const double &prev_size, const Map &map, const double &end_path_s){

  
    //Define the actual(x,y) points we will use for the planner
    Points next_vals;
  
   //Start with all of the previous path points from last time
    for(int i=0; i<previous_path.x.size(); ++i){
      next_vals.x.push_back(previous_path.x[i]);
      next_vals.y.push_back(previous_path.y[i]);
    }
  
  	//change last position to end of the path
  	SimpleCar refCar;
  	refCar.s = car.s;
    refCar.yaw = car.yaw;
  	refCar.x = car.x;
    refCar.y = car.y;
  
    if(prev_size > 0)
    {
      refCar.s = end_path_s;
    }
  
	// create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
    Points pts;

    // reference x,y, yaw states 
  	/*SimpleCar ref;
    ref.x = car.x;
    ref.y = car.y;
    ref.yaw = deg2rad(car.yaw);*/
  
    choosePoints(goal.lane, refCar, previous_path, prev_size, map, pts);
  	shiftPoints2CarReference(pts, refCar);
	
    //create a Spline
    tk::spline s;

    //set(x,y) points to the spline
    s.set_points(pts.x, pts.y);

	choosePointsAccordingSpeed_shift2MapReference_getTrajectory(goal.speed, s, refCar, previous_path, next_vals);
    
  return next_vals;
}


#endif  // TRAYECTORY_H