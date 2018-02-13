#pragma once

#include "utils.hpp"
#include "Map.h"

#include <tuple>
#include <vector>

struct CarState
{
  CarState(Point p, double yaw, FrenetPoint fp, double v) : position(p), yaw(yaw), position_frenet(fp), speed(v){};
  Point position;
  FrenetPoint position_frenet;
  double yaw;
  double speed;
};

class Behavior
{
public:
  Behavior(Map const &map);
  Path plan(CarState const &cs, Path const &previous_path, FrenetPoint end_point_frenet, Obstacles const &obstacles);

protected:
  Map const &_map;
};