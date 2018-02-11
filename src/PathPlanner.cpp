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
    FrenetPoint frenet_inc(cs.position_frenet);
    for (size_t i = 0; i < 50; i++)
    {
        frenet_inc.s += dist_inc;

        path.emplace_back(_map.toCartesian(frenet_inc));
    }

    return path;
}