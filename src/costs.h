#pragma once

#include "Map.h"
#include "utils.hpp"

#include "Eigen-3.3/Eigen/Core"

double inefficiency_cost(CarState const &cs, BehaviorState const &s, Path const &path, Obstacles const &obstacles, Map const &map)
{
    static double const LOOKBACK_DISTANCE = 20.;
    static double const MAX_DISTANCE = 100.;
    double min_speed = std::numeric_limits<double>::max();

    for (auto const &obst : obstacles)
    {
        if (obst.lane == s.wanted_lane)
        {
            // is obstacle ahead of us or besides us?
            double station_distance = obst.s - cs.position_frenet.s;
            map.normalizeRelativeStation(station_distance);

            // we dont care for obstacles behind us driving slower
            if (station_distance < 0 and station_distance > -LOOKBACK_DISTANCE and obst.speed < cs.speed)
            {
                continue;
            }

            if (station_distance < MAX_DISTANCE and station_distance > -LOOKBACK_DISTANCE and obst.speed < min_speed)
            {
                min_speed = obst.speed;
            }
        }
    }

    if (min_speed >= Map::MAX_LEGAL_SPEED)
    {
        return 0;
    }

    double relative_speed = min_speed / Map::MAX_LEGAL_SPEED;
    double cost = -std::pow(relative_speed, 2.) + 1;

    return cost;
}

double free_space_ahead_cost(CarState const &cs, BehaviorState const &s, Path const &path, Obstacles const &obstacles, Map const &map)
{
    double const speed_factor = 0.5 + (cs.speed / Map::MAX_LEGAL_SPEED);
    double const MAX_CONSIDERATION_DISTANCE = 100 * speed_factor; //m
    double const MIN_CLEARANCE_BACK = 5 * speed_factor;           //m

    std::vector<double> lane_distances;
    double min_distance = std::numeric_limits<double>::max();

    for (auto const &obst : obstacles)
    {
        if (obst.lane == s.wanted_lane)
        {
            double station_distance = obst.s - cs.position_frenet.s;
            map.normalizeRelativeStation(station_distance);

            if (station_distance > -MIN_CLEARANCE_BACK and station_distance < MAX_CONSIDERATION_DISTANCE)
            {
                lane_distances.push_back(station_distance);
            }
        }
    }

    // clear coast? lets do that
    if (lane_distances.empty())
    {
        return 0.;
    }

    double min_distance_ahead = *std::min_element(lane_distances.begin(), lane_distances.end());

    // obstacle besides us? not cheap ..
    if (min_distance_ahead < 0)
    {
        return 1.;
    }

    double relative_distance = min_distance_ahead / MAX_CONSIDERATION_DISTANCE;
    double cost = -std::pow(relative_distance, 2.) + 1;

    return cost;
}

double rechtsfahrgebot_cost(CarState const &cs, BehaviorState const &s, Path const &path, Obstacles const &obstacles, Map const &map)
{
    switch (s.wanted_lane)
    {
    // right-most lane, that's where we should drive
    case 2:
        return 0;
    // middle lane, okayish to be here
    case 1:
        return 0.1;
    // left-most lane, ony for takeover
    case 0:
        return 0.2;
    }

    return 0;
}

double safety_cost(CarState const &cs, BehaviorState const &s, Path const &path, Obstacles const &obstacles, Map const &map)
{
    // keeping the lane should always be safe
    if (s.maneuver == BehaviorState::KL)
    {
        return 0.;
    }

    double const speed_factor = 0.5 + (cs.speed / Map::MAX_LEGAL_SPEED);
    double const MAX_CONSIDERATION_DISTANCE = 30 * speed_factor; //m
    double const MIN_CLEARANCE_AHEAD = 15 * speed_factor;        //m
    double const MIN_CLEARANCE_BACK = 10 * speed_factor;         //m

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
    if (min_distance_abs > MAX_CONSIDERATION_DISTANCE)
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
    assert(bound < MAX_CONSIDERATION_DISTANCE);

    double relative_distance = (min_distance_abs - bound) / (MAX_CONSIDERATION_DISTANCE - bound);
    //double cost = 1.0 - (min_distance_abs - bound) / (MAX_CONSIDERATION_DISTANCE - bound);
    double cost = -std::pow(relative_distance, 2.) + 1;

    // cosine interpolation
    /*double y1 = 1;
    double y2 = 0;
    double mu = (min_distance_abs - bound) / (MAX_CONSIDERATION_DISTANCE - bound);
    double mu2 = (1 - std::cos(mu * M_PI)) / 2;
    double cost = (y1 * (1 - mu2) + y2 * mu2);
    */

    return cost;
}