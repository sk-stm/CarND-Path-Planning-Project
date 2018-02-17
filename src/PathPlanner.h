#pragma once

#include "utils.hpp"
#include "Map.h"
#include "PathPlanner.h"

class PathPlanner
{
  public:
    PathPlanner(Map const &map);
    Path plan(CarState const &cs, Path const &previous_path, BehaviorState const &planning_info);

  protected:
    Map const &_map;
};