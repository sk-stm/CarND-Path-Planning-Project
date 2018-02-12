#include "PathPlanner.h"

#include "Eigen-3.3/Eigen/Geometry"
#include "utils.hpp"
#include "spline.h"

#include <cmath>

PathPlanner::PathPlanner(Map const &map) : _map(map)
{
}

Path PathPlanner::plan(CarState const &cs, Path const &previous_path, FrenetPoint end_point_frenet, Obstacles const &obstacles)
{
    static double const SECONDS_PER_SAMPLING = 0.02; // s
    static double const MAX_LEGAL_SPEED = 22;        // m/s

    static int lane = 1;
    std::pair<double, double> our_lane_d_range = std::make_pair(Map::LANE_WIDTH * lane, Map::LANE_WIDTH * lane + Map::LANE_WIDTH);

    static double wanted_speed = cs.speed;
    bool is_obst_ahead_too_close = false;

    // obstacle stuff
    for (auto const &obst : obstacles)
    {
        double const obst_d = obst[6];
        bool const is_obst_on_my_lane = (obst_d >= our_lane_d_range.first) and (obst_d < our_lane_d_range.second);

        if (is_obst_on_my_lane)
        {
            Eigen::Vector2d const obst_speed_vec(obst[3], obst[4]);
            double const obst_speed = obst_speed_vec.norm();
            double obst_s = obst[5];

            // walkthrough magic:
            obst_s += (double)previous_path.size() * SECONDS_PER_SAMPLING * obst_speed;

            double const s_walkthrough_magic = (previous_path.empty()) ? cs.position_frenet.s : end_point_frenet.s;
            double const distance_walkthrough_magic = 30;
            is_obst_ahead_too_close = (obst_s > s_walkthrough_magic) and (obst_s < s_walkthrough_magic + distance_walkthrough_magic);

            if (is_obst_ahead_too_close)
            {
                break;
            }
        }
    }

    if (is_obst_ahead_too_close)
    {
        wanted_speed -= 0.44704;

        if (lane == 2)
        {
            // LCL
            lane = 1;
        }
        else if (lane == 1)
        {
            // LCL
            lane = 0;
        }
        else if (lane == 0)
        {
            // LCR
            lane = 1;
        }
    }
    else
    {
        wanted_speed += 0.44704;
    }

    if (wanted_speed > MAX_LEGAL_SPEED)
    {
        wanted_speed = MAX_LEGAL_SPEED;
    }
    else if (wanted_speed < 0)
    {
        wanted_speed = 0;
    }

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
    for (double distance : {30., 60., 90.})
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
    double const sampling_distance = target_dist / (SECONDS_PER_SAMPLING * wanted_speed);
    double const increment = target_point[0] / sampling_distance;
    double x_addon = 0;

    // pre-fill path with previous path (minus what the car travelled in the meantime)
    Path path = previous_path;

    // fill up the rest of the path
    int const points_to_fill_up = 50 - path.size();
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
    }

    return path;
}