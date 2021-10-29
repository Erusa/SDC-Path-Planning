#ifndef CAR_H
#define CAR_H

#include <vector>
#include <string>

struct Car{
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  int lane;
  string state;
};

struct SimpleCar{
  double x;
  double y;
  double yaw;
};

struct CarController{
	int lane;
  	double speed;
};

#endif