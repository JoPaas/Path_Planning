//
// Created by johannes on 13.11.18.
//

#include "vehicle.h"
#include "Constants.h"
//#include "jmt.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <iterator>
#include <random>
#include <algorithm>

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle() {

    observations = 0;
}

Vehicle::Vehicle(double s, double s_d, double s_dd, double d, double d_d, double d_dd) {

    state.s = s;         // s position
    state.s_d = s_d;       // s dot - velocity in s
    state.s_dd = s_dd;      // s dot-dot - acceleration in s
    state.d = d;         // d position
    state.d_d = d_d;       // d dot - velocity in d
    state.d_dd = d_dd;      // d dot-dot - acceleration in d
    observations = 0;

}

Vehicle::~Vehicle() {}

void Vehicle::update_maneuvers(const bool veh_left, const bool veh_right) {


    // update maneuvers
    // {go_left, stay_in_lane, go_right}
    maneuvers = {true, true, true};
    if (veh_left || lane == 0) maneuvers[0] = false;
    if (veh_right || lane == 2) maneuvers[2] = false;
}

void Vehicle::update_lane(){
    // update ego lane
    // 0: left lane, 1: middle lane, 2: right lane
    if (state.d < 3.0 * LANE_WIDTH) lane = 2;
    if (state.d < 2.0 * LANE_WIDTH) lane = 1;
    if (state.d < LANE_WIDTH) lane = 0;
}

void Vehicle::update_states(const double s, const double s_d, const double d, const double course, const double x, const double y) {

    state.s = s;
    state.s_d = s_d;
    state.s_dd = 0.0;
    state.d = d;
    state.d_d = 0.0;
    state.d_dd = 0.0;
    state.psi = course;
    state.x = x;
    state.y = y;
    dt = 0.0;


    if (observations > 0) { //1st derivative
        if (state.s < state_prev.s) {
            state_prev.s -= TRACK_LENGTH;
        }
        dt = (state.s - state_prev.s) / state.s_d;
        state.s_dd = (state.s_d - state_prev.s_d) / dt;

        state.d_d = (state.d - state_prev.d) / dt;
    }
    if (observations > 1) { //2nd derivative
        state.d_dd = (state.d_d - state_prev.d_d) / dt;
    }

    state_prev = state;
    observations++;
    if (observations > 2) observations = 2;
}