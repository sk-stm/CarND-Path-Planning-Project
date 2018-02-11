#pragma once

#include "utils.hpp"
#include "Map.h"

#include <tuple>
#include <vector>

struct CarState
{
<<<<<<< HEAD
    CarState(Point p, double yaw, FrenetPoint fp, double v) : position(p), yaw(yaw), position_frenet(fp), speed(v){};
=======
    CarState(Point p, double yaw, FrenetPoint fp, double v)
        : position(p), yaw(yaw), position_frenet(fp), speed(v){};
>>>>>>> bd4584b0c57c816cbbf2935bdae7abc64a7e7485
    Point position;
    FrenetPoint position_frenet;
    double yaw;
    double speed;
};

class PathPlanner
{
  public:
    PathPlanner(Map const &map);
<<<<<<< HEAD
    Path plan(CarState const &cs, Path const &previous_path, FrenetPoint end_point_frenet, Obstacles const &obstacles);
=======
    Path plan(CarState const &cs, Path const &previsous_path, FrenetPoint end_point_frenet, Obstacles const & obstacles);
>>>>>>> bd4584b0c57c816cbbf2935bdae7abc64a7e7485

  protected:
    Map const &_map;
};