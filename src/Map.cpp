#include "Map.h"

#include "spline.h"

#include <iostream>
#include <fstream>
#include <sstream>

Map::Map(std::string const &map_file, double max_s, Point map_center)
	: _max_s(max_s), _map_center(map_center)
{
	loadMap(map_file);
	smoothMap();
}

void Map::normalizeRelativeStation(double &station) const
{
	if (station > _max_s / 2)
	{
		station -= _max_s;
	}
	else if (station < -_max_s / 2)
	{
		station += _max_s;
	}
}

void Map::loadMap(std::string const &map_file)
{
	LOGGER->info("Loading map from '{}' ..", map_file);

	std::ifstream in_map(map_file.c_str(), std::ifstream::in);
	if (not in_map.is_open())
	{
		throw std::runtime_error("map file " + map_file + " does not exist");
	}

	std::string line;
	while (getline(in_map, line))
	{
		std::istringstream iss(line);
		Point p;
		FrenetMapPoint fmp;
		iss >> p[0];
		iss >> p[1];
		iss >> fmp.s;
		iss >> fmp.dVector[0];
		iss >> fmp.dVector[1];
		_map_waypoints_xy.push_back(p);
		_map_waypoints_frenet.push_back(fmp);
	}
	assert(_map_waypoints_xy.size() && "Map contains zero waypoints?");

	LOGGER->info(".. map loaded with {0:d} waypoints", _map_waypoints_xy.size());
}

void Map::smoothMap()
{
	LOGGER->info("Smoothing waypoints ..");
	std::vector<double> path_x, path_y, path_s, path_d0, path_d1;

	assert(not _map_waypoints_xy.empty());
	assert(_map_waypoints_xy.size() == _map_waypoints_frenet.size());

	// split data
	for (size_t i = 0; i < _map_waypoints_xy.size(); i++)
	{
		path_x.push_back(_map_waypoints_xy[i][0]);
		path_y.push_back(_map_waypoints_xy[i][1]);
		path_s.push_back(_map_waypoints_frenet[i].s);
		path_d0.push_back(_map_waypoints_frenet[i].dVector[0]);
		path_d1.push_back(_map_waypoints_frenet[i].dVector[1]);
	}

	// clear waypoints
	_map_waypoints_xy.clear();
	_map_waypoints_frenet.clear();

	// initialize splines
	tk::spline spline_x, spline_y, spline_d0, spline_d1;
	spline_x.set_points(path_s, path_x);
	spline_y.set_points(path_s, path_y);
	spline_d0.set_points(path_s, path_d0);
	spline_d1.set_points(path_s, path_d1);

	// sample splines and re-fill waypoints
	for (double s = 0; s < MAX_STATION; s += 1.0)
	{
		Point p;
		FrenetMapPoint fmp;

		p[0] = spline_x(s);
		p[1] = spline_y(s);

		fmp.s = s;
		fmp.dVector[0] = spline_d0(s);
		fmp.dVector[1] = spline_d1(s);

		_map_waypoints_xy.push_back(p);
		_map_waypoints_frenet.push_back(fmp);
	}

	LOGGER->info(".. map smoothed with {0:d} waypoints", _map_waypoints_xy.size());
}

size_t Map::closestWaypoint(Point const &p) const
{

	double closestLen = std::numeric_limits<double>::max();
	int closestWaypoint = 0;

	for (size_t i = 0; i < _map_waypoints_xy.size(); i++)
	{
		Point const &map_p = _map_waypoints_xy[i];
		double const dist = (map_p - p).norm();
		if (dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}

size_t Map::nextWaypoint(Point const &p, double theta) const
{

	size_t closestWaypointIdx = closestWaypoint(p);

	Point const &map_p = _map_waypoints_xy[closestWaypointIdx];
	Point const headingVector = map_p - p;
	double const headingAngle = atan2(headingVector[1], headingVector[0]);

	double angle = fabs(theta - headingAngle);
	angle = std::min(2 * M_PI - angle, angle);

	if (angle > M_PI / 4)
	{
		closestWaypointIdx++;
		if (closestWaypointIdx == _map_waypoints_xy.size())
		{
			closestWaypointIdx = 0;
		}
	}

	return closestWaypointIdx;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
FrenetPoint Map::toFrenet(Point const &p, double theta) const
{
	size_t next_wp = nextWaypoint(p, theta);
	size_t prev_wp;
	prev_wp = next_wp - 1;
	if (next_wp == 0)
	{
		prev_wp = _map_waypoints_xy.size() - 1;
	}

	Point n_p = _map_waypoints_xy[next_wp] - _map_waypoints_xy[prev_wp];
	Point dir = p - _map_waypoints_xy[prev_wp];

	// find the projection of x onto n
	double proj_norm = (dir.dot(n_p)) / (n_p.dot(n_p));
	Point proj = proj_norm * n_p;

	double frenet_d = (dir - proj).norm();

	//see if d value is positive or negative by comparing it to a center point
	Point d_center = _map_center - _map_waypoints_xy[prev_wp];
	double centerToPos = (d_center - dir).norm();
	double centerToRef = (d_center - proj).norm();

	if (centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for (size_t i = 0; i < prev_wp; i++)
	{
		frenet_s += (_map_waypoints_xy[i] - _map_waypoints_xy[i + 1]).norm();
	}

	frenet_s += (Point(0, 0) - proj).norm();

	return FrenetPoint(frenet_s, frenet_d);
}

// Transform from Frenet s,d coordinates to Cartesian x,y
Point Map::toCartesian(FrenetPoint fp) const
{
	int prev_wp = -1;

	while (fp.s > _map_waypoints_frenet[prev_wp + 1].s && (prev_wp < (int)(_map_waypoints_frenet.size() - 1)))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % _map_waypoints_xy.size();

	double heading = atan2((_map_waypoints_xy[wp2][1] - _map_waypoints_xy[prev_wp][1]), (_map_waypoints_xy[wp2][0] - _map_waypoints_xy[prev_wp][0]));
	// the x,y,s along the segment
	double seg_s = (fp.s - _map_waypoints_frenet[prev_wp].s);

	Point seg_point(_map_waypoints_xy[prev_wp][0] + seg_s * cos(heading), _map_waypoints_xy[prev_wp][1] + seg_s * sin(heading));

	double perp_heading = heading - M_PI / 2;

	Point p;
	p[0] = seg_point[0] + fp.d * cos(perp_heading);
	p[1] = seg_point[1] + fp.d * sin(perp_heading);

	return p;
}

int Map::getLeftLaneOf(int idx) const
{
	assert(idx >= 0 and idx < NUM_LANES);
	if (idx - 1 < 0)
	{
		return idx;
	}

	return idx - 1;
}

int Map::getRightLaneOf(int idx) const
{
	assert(idx >= 0 and idx < NUM_LANES);
	if (idx + 1 >= NUM_LANES)
	{
		return idx;
	}

	return idx + 1;
}