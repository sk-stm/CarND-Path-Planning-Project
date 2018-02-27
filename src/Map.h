#pragma once

#include "utils.hpp"

#include "spline.h"

#include <string>
#include <vector>

class Map
{
public:
  Map(std::string const &map_file, double max_s = 6945.554, Point map_center = Point(1000, 2000));
  size_t closestWaypoint(Point const &p) const;
  size_t nextWaypoint(Point const &p, double theta) const;
  FrenetPoint toFrenet(Point const &p, double theta) const;
  Point toCartesian(FrenetPoint fp) const;
  int getLeftLaneOf(int idx) const;
  int getRightLaneOf(int idx) const;
  void normalizeRelativeStation(double &station) const;
  void normalizeStation(double &station) const;
  int laneOfFrenetD(double d) const;

  static double constexpr LANE_WIDTH{4.};
  static double constexpr LANE_WIDTH_HALF{LANE_WIDTH / 2.};
  static size_t constexpr NUM_LANES{3};
  static double constexpr MAX_LEGAL_SPEED{22.};

protected:
  void loadMap(std::string const &mapFile);
  void smoothMap();

  // The max s value before wrapping around the track back to 0
  double const _max_s;
  Point const _map_center;

  std::vector<Point> _map_waypoints_xy;
  std::vector<FrenetMapPoint> _map_waypoints_frenet;
};