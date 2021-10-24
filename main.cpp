#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "trayectory.h"
#include "car.h"
#include "map.h"
#include "json.hpp"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  Map map;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map.waypoints_x.push_back(x);
    map.waypoints_y.push_back(y);
    map.waypoints_s.push_back(s);
    map.waypoints_dx.push_back(d_x);
    map.waypoints_dy.push_back(d_y);
  }
  
  //DONE: defined some variables
  int lane = 1; //starting on lane 1
  //Have a reference velocity to target
  double ref_vel = 0.0;
  

  h.onMessage([&ref_vel, &map,&lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          Car egoCar;
          egoCar.x = j[1]["x"];
          egoCar.y = j[1]["y"];
          egoCar.s = j[1]["s"];
          egoCar.d = j[1]["d"];
          egoCar.yaw = j[1]["yaw"];
          egoCar.speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          //Elsa:help to do transitions
          int prev_size = previous_path_x.size();

          json msgJson;


          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
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
          
          //Define the actual(x,y) points we will use for the planner
          vector < vector<double> > next_points;
          
          next_points = generateTrayectory(lane, ref_vel, egoCar, previous_path_x, previous_path_y, end_path_s, end_path_d, prev_size, map);
          
          msgJson["next_x"] = next_points[0];
          msgJson["next_y"] = next_points[1];

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}