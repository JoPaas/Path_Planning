//
// Created by johannes on 13.11.18.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#ifndef VEHICLE
#define VEHICLE

#include <vector>
#include <map>
#include <string>

using namespace std;

struct State {
    double s;
    double s_d;
    double s_dd;
    double d;
    double d_d;
    double d_dd;
    double psi;
    double psi_d;
    double x;
    double y;
};

class Vehicle {

private:

public:

    double dt;
    State state;
    State state_prev;
    int lane, lane_target;
    vector<bool> maneuvers;
    int observations;

    /**
    * Constructors
    */
    Vehicle();
    Vehicle(double s, double s_d, double s_dd, double d, double d_d, double d_dd);

    /**
    * Destructor
    */
    virtual ~Vehicle();

    void update_states(const double s, const double s_d, const double d, const double course, const double x, const double y);

    void update_maneuvers(const bool veh_left, const bool veh_right);

    void update_lane();

};

#endif

#endif //PATH_PLANNING_VEHICLE_H
