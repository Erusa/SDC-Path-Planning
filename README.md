# SDC-Path-Planning
My own Project from SDC Udacity Course (Project 7). This code programs the car to accelerate or change lines depending on cars nearby

*Simulator Requirement: Car is controlled by giving the point where it should be at certain time. From here, speed and accelaration are deduced.*

0. main.h --> main function which connects the car online
1. planning.h --> it decides the next state of the car, guided by the minimun cost. Then it gives trajectory.h the goal lane and goal speed of the car for next points.
2. trajectory.h --> it computes the future points of the car, using previous path and new desired lane and speed.
3. spline.h --> copy of the library spline.h
3. cost.h  ---> calculate cost of collision, change line, and speed
4. car.h, points.h and map.h --> contain simple structures to improve readability of the code

<p float="left">
<img src="./Picture.png" width="1000" height="600">
</p>

## planning.h ##
CarController choose_next_state(Car &car, const vector< vector<double> > &sensor_fusion, CarController &old_goal, const int &prev_size)
1. Succesor states are calculate  -->*successor_states(car)*
2. Each posible state and its cost are computed. ---> *calculate_cost(goal, sensor_fusion, car, prev_size)*
3. The state with the minimum cost is chosen.  ---> *min_element(begin(costs), end(costs))*
4. The chosen state gives the goal lane and goal speed  --> *final_goals[best_idx]*

## trajectory.h ##
Points generateTrajectory(const CarController &goal, const Car &car, const Points &previous_path, const double &prev_size, const Map &map, const double &end_path_s)
1. First, the function add the old point from previous_path to the new path. It assures a continoues path.
2. Second, the new points have to be generated:
  * First 5 Indicator Points are chosen, taking into account the **goal lane**  --->*choosePoints(goal.lane, refCar, previous_path, prev_size, map, pts)*
  * This points are shifted to car. This simplify the spline  --> *shiftPoints2CarReference(pts, refCar)*
  * A spline is computed with this points.  ---> *tk::spline s*
  * Points from the spline are chosen, according to **goal speed**, points are shiften back. --> *choosePointsAccordingSpeed_shift2MapReference_getTrajectory(goal.speed, s, refCar, previous_path, next_vals)*
  * New points are added to the new path
  
## cost.h ##
float calculate_cost(const CarController& goal, const vector< vector<double> > &sensor_fusion, const Car &car, const int &prev_size)  
1. collision is calculated using the predicted position of the ego car and car in the same line, when distance is lower than 60m 
  
  ---> *float collision_cost(const CarController& goal, const vector< vector<double> > &sensor_fusion, const Car &egoCar, const int &prev_size)*
  ```
  if(((check_car_s > car_s) && (check_car_s - car_s < 60)) || (abs(check_car_s - car_s) < 5 ))
        { 
          float new_cost = COLLISION*pow(60 - (check_car_s - car_s),2);
          cost = new_cost > cost ? new_cost : cost;
         }
  ```
2. it changes only if necessary
```  
float change_cost(const CarController& goal,  const Car &egoCar){ //max = 10
  float cost = (goal.lane == egoCar.lane ?  0: CHANGE);
  return cost;
}
```
