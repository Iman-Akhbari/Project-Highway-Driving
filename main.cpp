#include <uWS/uWS.h>
#include <fstream>
#include <math.h>
#include <chrono>
#include <vector>
#include <iostream>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
using namespace std;
using nlohmann::json;     
using std::string;
using std::vector;
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Frenet (s,d) to Cartesian (x,y)
vector<double> Frenet_to_Cartesian(double s, double d, const vector<double> &map_s, const vector<double> &map_x, const vector<double> &map_y)
{	
  int p_wp = -1;
	while(s > map_s[p_wp+1] && (p_wp < (int)(map_s.size()-1) ))
	{
		p_wp++;
	}
	double p_head = atan2((map_y[(p_wp+1)%map_x.size()]-map_y[p_wp]),(map_x[(p_wp+1)%map_x.size()]-map_x[p_wp]))-3.14159265359/2;
	double x = map_x[p_wp]+(s-map_s[p_wp])*cos(atan2((map_y[(p_wp+1)%map_x.size()]-map_y[p_wp]),(map_x[(p_wp+1)%map_x.size()]-map_x[p_wp]))) + d*cos(p_head);
	double y = map_y[p_wp]+(s-map_s[p_wp])*sin(atan2((map_y[(p_wp+1)%map_x.size()]-map_y[p_wp]),(map_x[(p_wp+1)%map_x.size()]-map_x[p_wp]))) + d*sin(p_head);
	return {x,y};
}
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
  ifstream in_map_(map_file_.c_str(), ifstream::in);
  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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
  double reference_s = 0.0; // Reference velocity miles per hour  
  int starting_lane = 1;                   // Assuming vehicles start from middle lane and assigning 1 to middle lane, (0 to left lane and 2 to right lane)
  h.onMessage([&reference_s, &starting_lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    ///auto sdata = string(data).substr(0, length);           
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];                    
            int prev_path_size = previous_path_x.size();   // recording previous path size.

            
            if (prev_path_size > 0) {               // This is to avoid accidents with other vehicles in the road
              car_s = end_path_s;
            }
            //  finding out other vehicles locations(lanes) in the road by prediction            
            bool vehicle_front = false;  
            bool vehicle_left = false;
            bool vehicle_right = false;            
            for ( int i = 0; i < sensor_fusion.size(); i++ ) {
                float d = sensor_fusion[i][6];
                int vehicle_lane = -1;               
                if ( d < 4  && d > 0 ) {   // checking against our own lane
                  vehicle_lane = 0;
                } else if ( d > 4 && d < 8 ) {
                  vehicle_lane = 1;
                } else if ( d > 8 && d < 12 ) {
                  vehicle_lane = 2;
                }
                if (vehicle_lane < 0) {
                  continue;
                }
                // getting vehicle speed
                double chck_vehicle_s = sensor_fusion[i][5];
                double S_y = sensor_fusion[i][4];  
                double S_x = sensor_fusion[i][3];                
                //double chck_s = sqrt(S_x*S_x + S_y*S_y);                
                // Finding vehicle coordinate s by executing prev trajectory
                chck_vehicle_s = chck_vehicle_s + ((double)prev_path_size*0.02*sqrt(S_x*S_x + S_y*S_y));

                if ( vehicle_lane == starting_lane ) { //what needs to be done if other vehicle is in our lane                  
                  vehicle_front |= chck_vehicle_s > car_s && chck_vehicle_s - car_s < 30;
                } else if ( vehicle_lane - starting_lane == -1 ) { //what needs to be done if other vehicle is in our left lane   
                  vehicle_left |= car_s - 30 < chck_vehicle_s && car_s + 30 > chck_vehicle_s;
                } else if ( vehicle_lane - starting_lane == 1 ) { //what needs to be done if other vehicle is in our right lane          
                  vehicle_right |= car_s - 30 < chck_vehicle_s && car_s + 30 > chck_vehicle_s;
                }
            }                      
            const double highest_speed = 49.5;                         
            const double highest_ACC = .224;  
            double s_delta = 0;
            if ( vehicle_front ) { // if there is a vehicle in fornt
              if ( !vehicle_left && starting_lane > 0 ) { // if there is a left lane, and its empty, then can change lane to left               
                starting_lane--; 
              } else if ( !vehicle_right && starting_lane != 2 ){ // if there is a right lane, and its empty, then can change lane to right                           
                starting_lane++; 
              } else {
                s_delta = s_delta - highest_ACC;
              }
            } else {
              if ( starting_lane != 1 ) { 
                if ( ( starting_lane == 0 && !vehicle_right ) || ( starting_lane == 2 && !vehicle_left ) ) {
                  starting_lane = 1; // if vehicle is not in center line and center line is empty, then vehicle can back to center line
                }
              }
              if ( reference_s < highest_speed ) {
                s_delta = s_delta + highest_ACC;
              }
            }
                    // vectors to record previous points coordinates
          	vector<double> points_x;
            vector<double> points_y;
            double reference_x = car_x;
            double reference_y = car_y; 
            double reference_yaw = car_yaw * 3.14159265359 / 180;           
            if ( prev_path_size < 2 ) {                
                double pre_vehicle_x = car_x - cos(car_yaw);
                double pre_vehicle_y = car_y - sin(car_yaw);
                points_x.push_back(pre_vehicle_x);
                points_x.push_back(car_x);
                points_y.push_back(pre_vehicle_y);
                points_y.push_back(car_y);
            } else {           
                reference_x = previous_path_x[prev_path_size - 1];
                reference_y = previous_path_y[prev_path_size - 1];
                double reference_x_prev = previous_path_x[prev_path_size - 2];
                double reference_y_prev = previous_path_y[prev_path_size - 2];
                reference_yaw = atan2(reference_y-reference_y_prev, reference_x-reference_x_prev);

                points_x.push_back(reference_x_prev);
                points_x.push_back(reference_x);

                points_y.push_back(reference_y_prev);
                points_y.push_back(reference_y);
            }
            // Transfering frenet to cartesian to set up future target points
            vector<double> future_waypoint0 = Frenet_to_Cartesian(car_s + 30, 2 + 4*starting_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> future_waypoint1 = Frenet_to_Cartesian(car_s + 60, 2 + 4*starting_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> future_waypoint2 = Frenet_to_Cartesian(car_s + 90, 2 + 4*starting_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            points_x.push_back(future_waypoint0[0]);
            points_x.push_back(future_waypoint1[0]);
            points_x.push_back(future_waypoint2[0]);

            points_y.push_back(future_waypoint0[1]);
            points_y.push_back(future_waypoint1[1]);
            points_y.push_back(future_waypoint2[1]);          
            for ( int i = 0; i < points_x.size(); i++ ) {       // Shifting to vehicle coordiantes
              double vehicle_coordinate_x = points_x[i] - reference_x;
              double vehicle_coordinate_y = points_y[i] - reference_y;
              points_x[i] = vehicle_coordinate_x * cos(0 - reference_yaw) - vehicle_coordinate_y * sin(0 - reference_yaw);
              points_y[i] = vehicle_coordinate_x * sin(0 - reference_yaw) + vehicle_coordinate_y * cos(0 - reference_yaw);
            }                      
            tk::spline s;      // building spline.
            s.set_points(points_x, points_y);
            // defining vectors for recording previous paths.
          	vector<double> next_value_x;
          	vector<double> next_value_y;
            for ( int i = 0; i < prev_path_size; i++ ) {
              next_value_x.push_back(previous_path_x[i]);
              next_value_y.push_back(previous_path_y[i]);
            }
            // Calculate distance y position on 30 m ahead.
            double buffer_x = 0;
            double t_x = 30.0;  
            double t_y = s(t_x);
            double t_d = sqrt(t_x*t_x + t_y*t_y);            
            for( int i = 1; i < 50 - prev_path_size; i++ ) {
              reference_s = reference_s + s_delta;
              if ( reference_s > highest_speed ) {
                reference_s = highest_speed;
              } else if ( reference_s < highest_ACC ) {
                reference_s = highest_ACC;
              }                   
              double x_p = buffer_x + t_x*(0.02*reference_s/2.24)/t_d;
              double y_p = s(x_p);
              buffer_x = x_p;
              double x_reference = x_p;
              double y_reference = y_p;
              x_p = x_reference * cos(reference_yaw) - y_reference * sin(reference_yaw);
              y_p = x_reference* sin(reference_yaw) + y_reference * cos(reference_yaw);
              x_p += reference_x;
              y_p += reference_y;
              next_value_x.push_back(x_p);
              next_value_y.push_back(y_p);
            }       
            json msgJson;
          	msgJson["next_x"] = next_value_x;
          	msgJson["next_y"] = next_value_y;
          	auto msg = "42[\"control\","+ msgJson.dump()+"]";
          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
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