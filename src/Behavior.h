#pragma once

#include "Map.h"
#include "PathPlanner.h"
#include "Obstacles.h"
#include "utils.hpp"

class Behavior
{
public:
  Behavior(Map const &map);
  Path plan(CarState const &cs, Path const &previous_path, FrenetPoint end_point_frenet, Obstacles const &obstacles);

protected:
  std::vector<BehaviorState> getPossibleManeuvers();
  double calculateCosts(CarState const &cs, BehaviorState const &s, Path const &path, Obstacles const &obstacles);

  PathPlanner _path_planner;
  Map const &_map;
  BehaviorState _state;
};