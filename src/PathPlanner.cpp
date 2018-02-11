#include "PathPlanner.h"

<<<<<<< HEAD
#include "Eigen-3.3/Eigen/Geometry"
#include "utils.hpp"
#include "spline.h"

=======
#include "utils.hpp"
>>>>>>> bd4584b0c57c816cbbf2935bdae7abc64a7e7485
#include <cmath>

PathPlanner::PathPlanner(Map const &map) : _map(map)
{
}

<<<<<<< HEAD
Path PathPlanner::plan(CarState const &cs, Path const &previous_path, FrenetPoint end_point_frenet, Obstacles const &obstacles)
{
    static double const SECONDS_PER_SAMPLING = 0.02;
    static double const REFERENCE_SPEED = 22;

    int lane = 1;

    Point ref_point;
    double ref_yaw;
    Path waypoint_path;

    // if previous path is almost empty, use car state as reference state
    if (previous_path.size() < 2)
    {
        Point prev_car = cs.position - Point(cos(cs.yaw), sin(cs.yaw));
        waypoint_path.push_back(prev_car);
        waypoint_path.push_back(cs.position);

        ref_point = cs.position;
        ref_yaw = cs.yaw;
    }
    // use previous path's end-point as reference state
    else
    {
        Point prev_ref = previous_path[previous_path.size() - 2];
        Point ref = previous_path[previous_path.size() - 1];
        Point ref_dir = ref - prev_ref;

        waypoint_path.push_back(prev_ref);
        waypoint_path.push_back(ref);

        ref_point = ref;
        ref_yaw = atan2(ref_dir[1], ref_dir[0]);
    }

    // in Frenet, add evenly 30m spaced points ahead of the starting reference
    for (double distance : {30., 60., 90., 120.})
    {
        FrenetPoint ref_point_frenet = _map.toFrenet(ref_point, ref_yaw);
        FrenetPoint next_point_frenet(ref_point_frenet.s + distance, 2 + 4 * lane);
        Point next_point = _map.toCartesian(next_point_frenet);
        waypoint_path.push_back(next_point);
    }

    // switch to vehicle-centered reference frame
    Eigen::Affine2d vehicle_map_transform(Eigen::Translation2d(ref_point) * Eigen::Rotation2Dd(ref_yaw));
    for (Point &p : waypoint_path)
    {
        p = vehicle_map_transform.inverse() * p;
    }

    // setup the cubic spline from waypoint_path
    std::vector<double> path_x, path_y;
    std::tie(path_x, path_y) = waypoint_path.split();
    tk::spline spline;
    spline.set_points(path_x, path_y);

    // calculate how to break up spline points so that we travel at our desired reference velocity
    double const target_x = 30;
    Point const target_point(target_x, spline(target_x));
    double const target_dist = target_point.norm();
    double const sampling_distance = target_dist / (SECONDS_PER_SAMPLING * REFERENCE_SPEED);
    double const increment = target_point[0] / sampling_distance;
    double x_addon = 0;

    // pre-fill path with previous path (minus what the car travelled in the meantime)
    Path path = previous_path;

    // fill up the rest of the path
    int const points_to_fill_up = 200 - path.size();
    for (size_t i = 0; i < points_to_fill_up; i++)
    {
        double x = x_addon + increment;
        if (x > path_x.back())
        {
            break;
        }

        double y = spline(x);
        x_addon = x;

        Point p(x, y);
        // switch back to map-centered reference frame
        p = vehicle_map_transform * p;
        path.push_back(p);
=======
Path PathPlanner::plan(CarState const &cs, Path const &previsous_path, FrenetPoint end_point_frenet, Obstacles const &obstacles)
{
    Path path;

    double dist_inc = 0.5;
    for (size_t i = 0; i < 50; i++)
    {
        path.emplace_back(
            cs.position[0] + (dist_inc * i) * cos(cs.yaw),
            cs.position[1] + (dist_inc * i) * sin(cs.yaw));
>>>>>>> bd4584b0c57c816cbbf2935bdae7abc64a7e7485
    }

    return path;
}