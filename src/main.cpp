#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "Constants.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }

double rad2deg(double x) { return x * 180 / pi(); }

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

double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for (int i = 0; i < maps_x.size(); i++) {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x, y, map_x, map_y);
        if (dist < closestLen) {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {

    int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y - y), (map_x - x));

    double angle = fabs(theta - heading);
    angle = min(2 * pi() - angle, angle);

    if (angle > pi() / 4) {
        closestWaypoint++;
        if (closestWaypoint == maps_x.size()) {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
    int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0) {
        prev_wp = maps_x.size() - 1;
    }

    double n_x = maps_x[next_wp] - maps_x[prev_wp];
    double n_y = maps_y[next_wp] - maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - maps_x[prev_wp];
    double center_y = 2000 - maps_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef) {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; i++) {
        frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double>
getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
    int prev_wp = -1;

    while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
        prev_wp++;
    }

    int wp2 = (prev_wp + 1) % maps_x.size();

    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - pi() / 2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x, y};

}

// Get the lane number of a car based on it's "s" value
int getLaneID(float d, float laneWidth = 4.0) {
    int lane;
    double rest = d - fmod(d, laneWidth);
    lane = rest / laneWidth;

    return lane;
}

int main() {
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    Vehicle ego_vehicle = Vehicle();

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";

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

    // V: Set target referent speed
    double ref_vel = 0;

    // State
    string state = "Drive";

    h.onMessage(
            [&ref_vel, &state, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &ego_vehicle](
                    uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                    uWS::OpCode opCode) {
                // "42" at the start of the message means there's a websocket message event.
                // The 4 signifies a websocket message
                // The 2 signifies a websocket event
                //auto sdata = string(data).substr(0, length);
                //cout << sdata << endl;
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
                            car_speed *= 0.44704; //[mi/h] to [m/s]

                            double max_acceleration = 1.0; //.224;
                            max_acceleration *= 0.44704; //[mi/h] to [m/s]

                            // update ego vehicle state
                            ego_vehicle.update_states(car_s, car_speed, car_d, deg2rad(car_yaw), car_x, car_y);
                            ego_vehicle.update_lane();

                            // cout<<"My Lane = "<<lane<<"\n";
                            if (state == "Drive" && ref_vel < SPEED_LIMIT) {
                                ref_vel += max_acceleration;
                            }

                            // Previous path is the piece of path that wasn't executed by the controller until NOW
                            // Previous path data given to the Planner
                            auto previous_path_x = j[1]["previous_path_x"];
                            auto previous_path_y = j[1]["previous_path_y"];
                            // Previous path's end s and d values
                            double end_path_s = j[1]["end_path_s"];
                            double end_path_d = j[1]["end_path_d"];

                            // Sensor Fusion Data, a list of all other cars on the same side of the road.
                            auto sensor_fusion = j[1]["sensor_fusion"];

                            // V: Save the previous size of the path
                            int prev_size = previous_path_x.size();

                            json msgJson;

                            float real_current_s = car_s;

                            if (prev_size > 0) {
                                car_s = end_path_s;
                            }

                            vector<double> other_data;

                            double distance_before_considering_lead_vehicle = ego_vehicle.state.s_d * 2.5; //2.0s distance



                            // data format for objects: [ id, x, y, vx, vy, s, d]
                            vector<Vehicle> objects;
                            for (auto obj : sensor_fusion) {
                                double obj_vel = sqrt(pow((double) obj[3], 2) + pow((double) obj[4], 2));
                                Vehicle object = Vehicle(obj[5], obj_vel, 0, obj[6], 0, 0);
                                object.update_lane();
                                objects.push_back(object);
                            }

                            Vehicle ACC_object;
                            double min_s_diff = distance_before_considering_lead_vehicle;
                            bool acc_active = false;
                            for (Vehicle obj : objects) {
                                double s_diff = obj.state.s - ego_vehicle.state.s;
                                double d_diff = obj.state.d - ego_vehicle.state.d;
                                if (obj.lane == ego_vehicle.lane && s_diff > 0.0 &&
                                    s_diff < distance_before_considering_lead_vehicle && s_diff < min_s_diff) {
                                    ACC_object = obj;
                                    min_s_diff = s_diff;
                                    acc_active = true;
                                    cout << "obj s_diff: " << s_diff << endl;
                                    std::cout << "ego_speed: " << ego_vehicle.state.s_d << " acc_speed: " << ACC_object.state.s_d << std::endl;
                                }
                            }

                            if (acc_active) {
                                state = "Follow";
                            } else {
                                state = "Drive";
                            }

                            double min_dist = ego_vehicle.state.s_d * 1.5; //1.5s distance
                            //std::cout << "min_dist = " << min_dist << ", acc_dist = " << ACC_object.state.s - ego_vehicle.state.s << std::endl;

                            double follow_speed = SPEED_LIMIT;
                            /*if (state == "Follow") {
                                follow_speed = ACC_object.state.s_d;
                                if (follow_speed < car_speed) {
                                    ref_vel -= max_acceleration;
                                } else if (follow_speed > car_speed && car_speed < SPEED_LIMIT) {
                                    ref_vel += max_acceleration;
                                }
                            }*/

                            if (state == "Follow") {
                                follow_speed = ACC_object.state.s_d;
                                double s_diff = ACC_object.state.s - ego_vehicle.state.s;
                                if (s_diff < min_dist) {
                                    ref_vel -= max_acceleration;
                                } else if (s_diff > min_dist && ego_vehicle.state.s_d < SPEED_LIMIT && ref_vel < SPEED_LIMIT) {
                                    //ref_vel = ego_vehicle.state.s_d + max_acceleration;
                                    ref_vel += max_acceleration;
                                }
                            }

                            // Until here, it's simply an ACC
                            // From now on is lane changing

                            bool veh_left = false, veh_right = false, veh_ahead = false;
                            for (Vehicle obj : objects) {
                                obj.update_lane();
                                double s_diff = obj.state.s - ego_vehicle.state.s;
                                double d_diff = obj.state.d - ego_vehicle.state.d;
                                if (s_diff < distance_before_considering_lead_vehicle && s_diff > ego_vehicle.state.s_d * -0.5) {
                                    if (obj.lane == ego_vehicle.lane + 1) {
                                        veh_right = true;

                                    } else if (obj.lane == ego_vehicle.lane - 1) {
                                        veh_left = true;

                                    } else if (obj.lane == ego_vehicle.lane) {
                                        veh_ahead = true;
                                    }
                                }
                            }


                            //cout << "{ " << veh_left << " | " << veh_ahead << " | " << veh_right << " }" << endl;

                            ego_vehicle.update_maneuvers(veh_left, veh_right);

                            cout << "maneuvers: {" << ego_vehicle.maneuvers[0] << " | " << ego_vehicle.maneuvers[1]
                                 << " | " << ego_vehicle.maneuvers[2] << " }" << endl;

                            static bool LC_left, LC_right;

                            // reset lane change states when arrived in new lane
                            if ((LC_left || LC_right) &&
                                fabs(ego_vehicle.state.d -
                                     (LANE_WIDTH * ego_vehicle.lane_target + LANE_WIDTH / 2.0)) <
                                LANE_TRESH) {
                                LC_left = false;
                                LC_right = false;
                                //cout << "LC left done" << endl;
                            }

                            // change lane if vehicle ahead
                            if (!LC_left && !LC_right) {
                                int lc_dir = 1 - ego_vehicle.lane;
                                if (veh_ahead && ego_vehicle.maneuvers[0]) {
                                    LC_left = true;
                                    //cout << "LC left" << endl;
                                    ego_vehicle.lane_target = ego_vehicle.lane - 1;
                                } else if (veh_ahead && ego_vehicle.maneuvers[2]) {
                                    LC_right = true;
                                    //cout << "LC right" << endl;
                                    ego_vehicle.lane_target = ego_vehicle.lane + 1;
                                } else if (lc_dir != 0 && ego_vehicle.maneuvers[1 + lc_dir]) {
                                    if (lc_dir > 0) {
                                        LC_right = true;
                                        //cout << "LC right" << endl;
                                    } else {
                                        LC_left = true;
                                        //cout << "LC left" << endl;
                                    }
                                    ego_vehicle.lane_target = 1;
                                } else ego_vehicle.lane_target = ego_vehicle.lane;
                            }

                            cout << "lane: " << ego_vehicle.lane << endl;
                            cout << "lane target: " << ego_vehicle.lane_target << endl;
                            cout << "LC left: " << LC_left << " LC right: " << LC_right << endl;

                            /*static int lc_cooldown = 0;
                            std::cout << "lc_cooldown: " << lc_cooldown << std::endl;
                            if (veh_ahead && follow_speed < SPEED_LIMIT && lc_cooldown == 0){
                                // change lane
                                if (ego_vehicle.maneuvers[0]){
                                    lane -= 1;
                                    lc_cooldown = 30;
                                }
                                else if (ego_vehicle.maneuvers[2]){
                                    lane += 1;
                                    lc_cooldown = 30;
                                }
                            }

                            if (lc_cooldown > 0) lc_cooldown--;*/

                            // V: Create a spline through distant points
                            vector<double> next_x_vals;
                            vector<double> next_y_vals;

                            vector<double> ptsx;
                            vector<double> ptsy;

                            double ref_x = car_x;
                            double ref_y = car_y;
                            double ref_yaw = car_yaw;

                            if (prev_size < 2) {
                                double prev_car_x = car_x - cos(car_yaw);
                                double prev_car_y = car_y - sin(car_yaw);

                                next_x_vals.push_back(prev_car_x);
                                next_x_vals.push_back(car_x);

                                next_y_vals.push_back(prev_car_y);
                                next_y_vals.push_back(car_y);
                            } else {
                                ref_x = previous_path_x[prev_size - 1];
                                ref_y = previous_path_y[prev_size - 1];

                                double ref_x_prev = previous_path_x[prev_size - 2];
                                double ref_y_prev = previous_path_y[prev_size - 2];
                                ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                                ptsx.push_back(ref_x_prev);
                                ptsx.push_back(ref_x);

                                ptsy.push_back(ref_y_prev);
                                ptsy.push_back(ref_y);
                            }

                            vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * ego_vehicle.lane_target), map_waypoints_s,
                                                            map_waypoints_x, map_waypoints_y);
                            vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * ego_vehicle.lane_target), map_waypoints_s,
                                                            map_waypoints_x, map_waypoints_y);
                            vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * ego_vehicle.lane_target), map_waypoints_s,
                                                            map_waypoints_x, map_waypoints_y);

                            ptsx.push_back(next_wp0[0]);
                            ptsx.push_back(next_wp1[0]);
                            ptsx.push_back(next_wp2[0]);

                            ptsy.push_back(next_wp0[1]);
                            ptsy.push_back(next_wp1[1]);
                            ptsy.push_back(next_wp2[1]);

                            for (int i = 0; i < ptsx.size(); i++) {
                                double shift_x = ptsx[i] - ref_x;
                                double shift_y = ptsy[i] - ref_y;

                                ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
                                ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
                            }

                            tk::spline s;

                            s.set_points(ptsx, ptsy);

                            for (int i = 0; i < previous_path_x.size(); i++) {
                                next_x_vals.push_back(previous_path_x[i]);
                                next_y_vals.push_back(previous_path_y[i]);
                            }

                            double target_x = 30.0;
                            double target_y = s(target_x);
                            double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

                            double x_add_on = 0;

                            for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
                                double N = (target_dist / (.02 * ref_vel));
                                double x_point = x_add_on + (target_x) / N;
                                double y_point = s(x_point);

                                x_add_on = x_point;

                                double x_ref = x_point;
                                double y_ref = y_point;

                                x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
                                y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

                                x_point += ref_x;
                                y_point += ref_y;

                                next_x_vals.push_back(x_point);
                                next_y_vals.push_back(y_point);
                            }


                            // END
                            msgJson["next_x"] = next_x_vals;
                            msgJson["next_y"] = next_y_vals;

                            auto msg = "42[\"control\"," + msgJson.dump() + "]";

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

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

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
