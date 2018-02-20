#pragma once

#include "Map.h"
#include "utils.hpp"

#include "Eigen-3.3/Eigen/Core"

double inefficiency_cost(CarState const &cs, BehaviorState const &s, Path const &path, Obstacles const &obstacles, Map const &map)
{
    static double const LOOKBACK_DISTANCE = 20.;
    static double const MAX_DISTANCE = 150.;
    double min_speed = std::numeric_limits<double>::max();

    for (auto const &obst : obstacles)
    {
        if (obst.lane == s.wanted_lane)
        {
            // is obstacle ahead of us or besides us?
            double distance = obst.s - cs.position_frenet.s;
            map.normalizeRelativeStation(distance);

            if (distance < MAX_DISTANCE and distance > -LOOKBACK_DISTANCE and obst.speed < min_speed)
            {
                min_speed = obst.speed;
            }
        }
    }
    double const max_speed = std::max(Map::MAX_LEGAL_SPEED, min_speed);
    double const cost = (max_speed - min_speed) / max_speed;
    return cost;
}

double safety_cost(CarState const &cs, BehaviorState const &s, Path const &path, Obstacles const &obstacles, Map const &map)
{
    // keeping the lane should always be safe
    if (s.maneuver == BehaviorState::KL)
    {
        return 0.;
    }

    double const MIN_CONSIDERATION_DISTANCE = 50; //m
    double const MIN_CLEARANCE_AHEAD = 20;        //m
    double const MIN_CLEARANCE_BACK = 10;         //m

    double min_distance_abs = std::numeric_limits<double>::max();
    double min_distance_signed = 0;

    for (auto const &obst : obstacles)
    {
        if (obst.lane == s.wanted_lane)
        {
            double station_distance = obst.s - cs.position_frenet.s;
            map.normalizeRelativeStation(station_distance);
            double abs_distance = std::abs(station_distance);

            if (abs_distance < min_distance_abs)
            {
                min_distance_abs = abs_distance;
                min_distance_signed = station_distance;
            }
        }
    }

    // no obstacle in sight?
    if (min_distance_abs > MIN_CONSIDERATION_DISTANCE)
    {
        return 0.;
    }

    // obstacle within critical distance? assign max cost
    if (min_distance_signed > -MIN_CLEARANCE_BACK and min_distance_signed < MIN_CLEARANCE_AHEAD)
    {
        return 1.;
    }

    // interpolate within bounds
    double const bound = min_distance_signed < 0 ? MIN_CLEARANCE_BACK : MIN_CLEARANCE_AHEAD;
    assert(bound < MIN_CONSIDERATION_DISTANCE);

    double relative_distance = (min_distance_abs - bound) / (MIN_CONSIDERATION_DISTANCE - bound);
    //double cost = 1.0 - (min_distance_abs - bound) / (MIN_CONSIDERATION_DISTANCE - bound);
    double cost = -std::pow(relative_distance, 2.) + 1;

    // cosine interpolation
    /*double y1 = 1;
    double y2 = 0;
    double mu = (min_distance_abs - bound) / (MIN_CONSIDERATION_DISTANCE - bound);
    double mu2 = (1 - std::cos(mu * M_PI)) / 2;
    double cost = (y1 * (1 - mu2) + y2 * mu2);
    */

    return cost;
}