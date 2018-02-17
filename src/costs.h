#pragma once

#include "Map.h"
#include "utils.hpp"

#include "Eigen-3.3/Eigen/Core"

double lane_speed_ahead(size_t lane, double vehicle_s, Obstacles const &obstacles)
{
    Eigen::ArrayXd obst_speeds;
    for (auto const &obst : obstacles)
    {
        if (obst.lane == lane)
        {
            // is obstacle ahead of us?
            if (obst.s >= vehicle_s )
            {
                obst_speeds += obst.speed;
            }
        }
    }

    if(obst_speeds.size() == 0) {
        return Map::MAX_LEGAL_SPEED;
    } else {
        return obst_speeds.mean();
    }
}

double inefficiency_cost(CarState const &cs, BehaviorState const &s, Path const &path, Obstacles const &obstacles)
{
    int const wanted_lane   = s.wanted_lane;
    double const lane_speed = lane_speed_ahead(s.wanted_lane, cs.position_frenet.s, obstacles);
    assert(lane_speed >= 0 and lane_speed <= Map::MAX_LEGAL_SPEED);
    
    double const cost = (Map::MAX_LEGAL_SPEED - lane_speed / Map::MAX_LEGAL_SPEED);
    return cost;
}

double safety_cost(CarState const &cs, BehaviorState const &s, Path const &path, Obstacles const &obstacles) {
    double const MIN_CLEARANCE_AHEAD = 20; //m
    double const MIN_CLEARANCE_BACK = 30;

    double shortest_dist_back = std::numeric_limits<double>::max();
    double shortest_dist_ahead = std::numeric_limits<double>::max();

    for (auto const &obst : obstacles)
    {
        if (obst.lane == s.wanted_lane)
        {
            double station_distance = abs(obst.s - cs.position_frenet.s);
            // is obstacle ahead of us?
            if (obst.s >= cs.position_frenet.s && station_distance < shortest_dist_ahead)
            {
                shortest_dist_ahead = station_distance;
            } else if (obst.s < cs.position_frenet.s  && station_distance < shortest_dist_back) {
                shortest_dist_back = station_distance;
            }
        }
    }

    double const front_clearance_cost = ((tanh(-(shortest_dist_ahead-MIN_CLEARANCE_AHEAD)/(0.5*MIN_CLEARANCE_AHEAD)))+1.) / 2.;
    double const back_clearance_cost = ((tanh(-(shortest_dist_back-MIN_CLEARANCE_BACK)/(0.5*MIN_CLEARANCE_BACK)))+1.) / 2.;
    double const cost = std::min(front_clearance_cost, back_clearance_cost);

    return cost;
}