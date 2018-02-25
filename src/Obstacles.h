#pragma once

#include "Map.h"

struct Obstacle
{
    Obstacle(std::vector<double> const &data, size_t lane)
        : id(data[0]), x(data[1]), y(data[2]), s(data[5]), d(data[6]), speed_vec(data[3], data[4]), lane(lane)
    {
        speed = speed_vec.norm();
    }

    double id, x, y, s, d, speed;
    Eigen::Vector2d speed_vec;
    size_t lane;
};

class Obstacles : public std::vector<Obstacle>
{
  public:
    Obstacles(std::vector<std::vector<double>> const &data, Map const &map)
    {
        for (auto const &obst_data : data)
        {
            double const d = obst_data[6];
            int const lane = map.laneOfFrenetD(d);

            if (lane < 0 or lane >= Map::NUM_LANES)
            {
                // obstacle is somewhere outside the map?
                LOGGER->warn("there is an obstacle outside the map?");
                continue;
            }

            emplace_back(obst_data, lane);
        }
    }
};