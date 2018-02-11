#pragma once

#include "utils.hpp"
#include "Map.h"

#include <tuple>
#include <vector>

struct CarState
{
    CarState(Point p, double yaw, FrenetPoint fp, double v)
        : position(p), yaw(yaw), position_frenet(fp), speed(v){};
    Point position;
    FrenetPoint position_frenet;
    double yaw;
    double speed;
};

class PathPlanner
{
  public:
    PathPlanner(Map const &map);
    Path plan(CarState const &cs, Path const &previsous_path, FrenetPoint end_point_frenet, Obstacles const & obstacles);

  protected:
    Map const &_map;
};