#pragma once

#include "utils.hpp"

#include <string>
#include <vector>

class Map
{
public:
<<<<<<< HEAD
  Map(std::string const &map_file);
=======
  Map(std::string const &mapFile);
>>>>>>> bd4584b0c57c816cbbf2935bdae7abc64a7e7485
  size_t closestWaypoint(Point const &p) const;
  size_t nextWaypoint(Point const &p, double theta) const;
  FrenetPoint toFrenet(Point const &p, double theta) const;
  Point toCartesian(FrenetPoint fp) const;

protected:
  void loadMap(std::string const &mapFile);

  // The max s value before wrapping around the track back to 0
  double const _max_s{6945.554};
  Point const _map_center{1000, 2000};

  std::vector<Point> _map_waypoints_xy;
  std::vector<FrenetMapPoint> _map_waypoints_frenet;
};