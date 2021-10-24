#ifndef MAP_H
#define MAP_H


#include <math.h>
#include <string>
#include <vector>

using std::vector;

struct Map{
  vector<double> waypoints_x;
  vector<double> waypoints_y;
  vector<double> waypoints_s;
  vector<double> waypoints_dx;
  vector<double> waypoints_dy;
};
#endif