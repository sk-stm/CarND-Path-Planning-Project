#pragma once

#include "Map.h"
#include "utils.hpp"

#include "Eigen-3.3/Eigen/Core"

double inefficiency_cost(CarState const &cs, BehaviorState const &s, Path const &path, Obstacles const &obstacles)
{
    static double const LOOKBACK_DISTANCE = 10.;
    static double const MAX_DISTANCE = 150.;
    std::vector<double> obst_speeds;
    std::vector<double> obst_weights;

    for (auto const &obst : obstacles)
    {
        if (obst.lane == s.wanted_lane)
        {
            // is obstacle ahead of us or besides us?
            double distance = obst.s - cs.position_frenet.s;

            if (distance < MAX_DISTANCE and distance > -LOOKBACK_DISTANCE)
            {
                obst_speeds.push_back(obst.speed);
                obst_weights.push_back(1. - (distance + LOOKBACK_DISTANCE) / (MAX_DISTANCE + LOOKBACK_DISTANCE));
            }
        }
    }

    double average_weighted_speed;

    if (obst_speeds.size() == 0)
    {
        average_weighted_speed = Map::MAX_LEGAL_SPEED;
    }
    else
    {
        Eigen::VectorXd speeds = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(obst_speeds.data(), obst_speeds.size());
        Eigen::VectorXd weights = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(obst_weights.data(), obst_weights.size());
        weights = weights / weights.sum();
        average_weighted_speed = speeds.dot(weights);
    }

    assert(average_weighted_speed >= 0 and average_weighted_speed <= Map::MAX_LEGAL_SPEED);

    double const cost = (Map::MAX_LEGAL_SPEED - average_weighted_speed) / Map::MAX_LEGAL_SPEED;
    return cost;
}

double safety_cost(CarState const &cs, BehaviorState const &s, Path const &path, Obstacles const &obstacles)
{
    double const MIN_CLEARANCE_AHEAD = 40; //m
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
            }
            else if (obst.s < cs.position_frenet.s && station_distance < shortest_dist_back)
            {
                shortest_dist_back = station_distance;
            }
        }
    }

    double const front_clearance_cost = ((tanh(-(shortest_dist_ahead - MIN_CLEARANCE_AHEAD) / (0.5 * MIN_CLEARANCE_AHEAD))) + 1.) / 2.;
    double const back_clearance_cost = ((tanh(-(shortest_dist_back - MIN_CLEARANCE_BACK) / (0.5 * MIN_CLEARANCE_BACK))) + 1.) / 2.;
    double const cost = std::min(front_clearance_cost, back_clearance_cost);

    return cost;
}