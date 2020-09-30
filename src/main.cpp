/*
cd CarND-Path-Planning-Project
mkdir build && cd build
cmake .. && make
./path_planning
*/
#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

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
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          // The data format for each car is: [ id, x, y, vx, vy, s, d]
          auto sensor_fusion = j[1]["sensor_fusion"];

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          int previous_path_size = previous_path_x.size();
          
          //Start in lane 1
          static int lane = 1;
          
          //Have a reference velocity to target
          static double ref_vel = 0.0;//mph
          /****************************************************************************
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           **************************************************************************/
          
          /*************************************************************************
          * SENSOR FUSION
          *************************************************************************/
          if(previous_path_size > 0)
          {
          	car_s = end_path_s;
          }
          bool too_close = false;
          bool left_lane_is_empty = true;
          bool right_lane_is_empty = true;
          /*The data format for each car is: [ id, x, y, vx, vy, s, d]
          		id is a unique identifier for that car 
                x, y values are in global map coordinates
                vx, vy values are the velocity components (reference to the global map)
                s and d are the Frenet coordinates for that car.
          */
          for(int i = 0; i < sensor_fusion.size(); i++)
          {
          	
            float check_other_car_d = sensor_fusion[i][6];
            double check_other_car_vx = sensor_fusion[i][3];
            double check_other_car_vy = sensor_fusion[i][4];
            //magnitud of velocity on x and y
            double check_other_car_speed = sqrt(check_other_car_vx * check_other_car_vx + check_other_car_vy * check_other_car_vy);
            double check_other_car_s = sensor_fusion[i][5];
            
            /*Middle point of car's lane is 'lane' + 2 and 'lane' size is 4.*/
            if(check_other_car_d < (2+4*lane+2) && check_other_car_d > (2+4*lane-2))
            {
              	/*car is in my lane*/            	
              
              	check_other_car_s += ((double)previous_path_size * 0.02 * check_other_car_speed);
              	//check s values greater than mine and s gap
              	if((check_other_car_s > car_s) && ((check_other_car_s - car_s) < 30/*mts*/))
                {
                	//Reduce velocity so we don't crash
                  	//flag to try to change lanes                  	
                    too_close = true;                  	
                  	              
                }
            }            
            /*The d vector has a magnitude of 1 and points perpendicular to the road in the direction of the right-hand side of the road.
            Each lane is 4 m wide and our car should only ever be in one of the 3 lanes on the right-hand side.
            Lane 0: High speed - Lane 1: meddle - Lane 2: Low speed
            */  
            /*Check left lane allways*/
            if((lane > 0) 
                    /*is the other car in between of the next left lane left boundary*/
                    && check_other_car_d > (/*left lane = */(lane - 1)*/*lane width = */4)
                    /*...and the next left lane right boundary*/
                    && check_other_car_d < (/*left lane = */(lane - 1)*/*lane width = */4) + 4)
            {
              	//check s position in between +10mts and -30mts of our car
              	if((check_other_car_s < (car_s + 10/*mts*/)) && (check_other_car_s > (car_s - 30/*mts*/)))
                {
                	//Another car is in this lane
            		left_lane_is_empty = false;
                }
              	
            }
            /*Check right lane allways*/
            if((lane < 2) 
                    /*is the other car in between of the next right lane left boundary*/
                    && check_other_car_d > (/*right lane = */(lane + 1)*/*lane width = */4)
                    /*...and the next right lane right boundary*/
                    && check_other_car_d < (/*right lane = */(lane + 1)*/*lane width = */4) + 4)
            {
              	//check s position in between +10mts and -30mts of our car
              	if((check_other_car_s < (car_s + 10/*mts*/)) && (check_other_car_s > (car_s - 30/*mts*/)))
                {
                	//Another car is in this lane
            		right_lane_is_empty = false;
                }
              	
            }
          }
          
          if(too_close)
          {
          	if(lane > 0 && left_lane_is_empty)
              lane--;//change to the left lane if it's empty
            else if(lane < 2 && right_lane_is_empty)
              lane++;//change to the right lane if it's empty
            
            //If there are cars in front of us start breaking
            ref_vel -= 0.224;//des-Accel 5 mts/s*s 
          }
          else if(ref_vel < 49.5){
            //If no cars in front of us accelerate 5 mts/s*s 
            ref_vel += 0.224;
          }
          
            
          
          
          /*************************************************************************
          * DRIVE SMOOTH
          *
          * Use Cubic Spline interpolation in C++
          *************************************************************************/
          
          
          //Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          vector<double> ptsx;
          vector<double> ptsy;
          //Reference x, y, yaw states
          double ref_x;
          double ref_y;
          double ref_x_prev;
          double ref_y_prev;
          double ref_yaw;
          
          
          if(previous_path_size < 2)
          {
            ref_x = car_x;
          	ref_y = car_y;
            
            //If we are just starting, use two points that make the path tangent to the car
          	ref_x_prev = car_x - cos(car_yaw);
            ref_y_prev = car_y - sin(car_yaw);
            
            ref_yaw = deg2rad(car_yaw);
            
          }
          else
          {
          	//use the previous path's end point as starting reference
            ref_x = previous_path_x[previous_path_size - 1];
          	ref_y = previous_path_y[previous_path_size - 1];
            
            ref_x_prev = previous_path_x[previous_path_size - 2];
          	ref_y_prev = previous_path_y[previous_path_size - 2];
            
            ref_yaw = atan2(ref_y - ref_y_prev,ref_x - ref_x_prev);
            
            //use two points that make the path tangent to the previous path's end points

          }
          ptsx.push_back(ref_x_prev);
          ptsx.push_back(ref_x);

          ptsy.push_back(ref_y_prev);
          ptsy.push_back(ref_y);
          
          //In Frenet add evenly 30m space points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane), 
                                                   map_waypoints_s, 
                     								map_waypoints_x, 
                     									map_waypoints_y); 
          vector<double> next_wp1 = getXY(car_s + 60, (2+4*lane), 
                                                   map_waypoints_s, 
                     								map_waypoints_x, 
                     									map_waypoints_y); 
          vector<double> next_wp2 = getXY(car_s + 90, (2+4*lane), 
                                                   map_waypoints_s, 
                     								map_waypoints_x, 
                     									map_waypoints_y); 
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          for (int i = 0; i < ptsy.size(); ++i) {
              	double shift_x = ptsx[i] - ref_x;
              	double shift_y = ptsy[i] - ref_y;
              	
            	ptsx[i] = (shift_x * cos( 0 - ref_yaw)- shift_y* sin(0-ref_yaw));
            	ptsy[i] = (shift_x * sin( 0 - ref_yaw)+ shift_y* cos(0-ref_yaw));            
          }
          //Create a spline
          tk::spline s;
          
          //set (x,y) points to the spline
          s.set_points(ptsx,ptsy);          

          
          //Start with all of the previous path points from last time
          for(int i = 0; i < previous_path_size; i++)
          {
          	next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          //Calculate how to break up spline so that we travel at our desired reference velocity
          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x*target_x)+(target_y*target_y));
          double x_add_on = 0;
          
          for(int i = 1; i <= 50 - previous_path_size; i++)
          {
          	double N = (target_dist/(0.02*ref_vel/2.24));//mph -> mts/sec
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            //rotate back to normal after rotating it earlier
            x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
            
          }
          /**************************************************/
          json msgJson;
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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