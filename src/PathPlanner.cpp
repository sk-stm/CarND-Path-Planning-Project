#include "PathPlanner.h"

#include "utils.hpp"
#include <cmath>

PathPlanner::PathPlanner(Map const &map) : _map(map)
{
}

Path PathPlanner::plan(CarState const &cs, Path const &previsous_path, FrenetPoint end_point_frenet, Obstacles const &obstacles)
{
    Path path;

    double dist_inc = 0.5;
    for (size_t i = 0; i < 50; i++)
    {
        path.emplace_back(
            cs.position[0] + (dist_inc * i) * cos(cs.yaw),
            cs.position[1] + (dist_inc * i) * sin(cs.yaw));
    }

    return path;
}