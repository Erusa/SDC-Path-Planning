# SDC-Path-Planning
My own Project from SDC Udacity Course (Project 7). This code programs the car to accelerate or change lines depending on cars nearby

*Simulator Requirement: Car is controlled by giving the point where it should be at certain time. From here, speed and accelaration are deduced.*

0. main.h --> main function which connects the car online
1. planning.h --> it decides the next state of the car, guided by the minimun cost. Then it gives trajectory.h the goal lane and goal speed of the car for next points.
2. trajectory.h --> it computes the future points of the car, using previous path and new desired lane and speed.
3. spline.h --> copy of the library spline.h
3. cost.h  ---> calculate cost of collision, change line, and speed
4. car.h, points.h and map.h --> contain simple structures to improve readability of the code



### planning.h ###
CarController choose_next_state(Car &car, const vector< vector<double> > &sensor_fusion, CarController &old_goal, const int &prev_size)
1. Succesor states are calculate
2. Each posible state and its cost are computed.
3. The state with the minimum cost is chosen. 
4. The chosen state gives the goal lane and goal speed

### trajectory.h ###
Points generateTrajectory(const CarController &goal, const Car &car, const Points &previous_path, const double &prev_size, const Map &map, const double &end_path_s)
1. First, the function add the old point from previous_path to the new path. It assures a continoues path.
