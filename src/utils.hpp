#pragma once

#include "Eigen-3.3/Eigen/Core"
#include "spdlog/spdlog.h"

#include <cmath>
#include <vector>

#define LOGGER spdlog::get("console")

static double const SECONDS_PER_SAMPLING = 0.02; // s
using Point = Eigen::Vector2d;

struct Path : public std::vector<Point>
{
    Path() : std::vector<Point>(0) {}

    Path(std::vector<double> const &x_components, std::vector<double> const &y_components) : std::vector<Point>(x_components.size())
    {
        assert(x_components.size() == y_components.size());
        for (size_t i = 0; i < x_components.size(); i++)
        {
            (*this)[i] = Point(x_components[i], y_components[i]);
        }
    }

    std::tuple<std::vector<double>, std::vector<double>> split()
    {
        std::vector<double> x(this->size()), y(this->size());

        for (size_t i = 0; i < this->size(); i++)
        {
            x[i] = this->at(i)[0];
            y[i] = this->at(i)[1];
        }

        return std::make_tuple(std::move(x), std::move(y));
    }
};

struct BehaviorState
{
    enum Maneuver
    {
        KL,
        LCL,
        LCR
    };
    int wanted_lane{1};
    int current_lane{1};
    double wanted_speed{0};
    Maneuver maneuver{Maneuver::KL};

    std::string to_string() const
    {
        switch (maneuver)
        {
        case KL:
            return "KL";
        case LCL:
            return "LCL";
        case LCR:
            return "LCR";
        }
    };
};

struct FrenetPoint
{
    FrenetPoint() : FrenetPoint(0, 0){};
    FrenetPoint(double s, double d) : s(s), d(d){};
    double s;
    double d;
};

struct FrenetMapPoint
{
    FrenetMapPoint() : FrenetMapPoint(0, Point::Zero()){};
    FrenetMapPoint(double s, Point const &d) : s(s), dVector(d){};
    double s;
    Point dVector;
};

struct CarState
{
    CarState(Point p, double yaw, FrenetPoint fp, double v) : position(p), yaw(yaw), position_frenet(fp), speed(v){};
    Point position;
    FrenetPoint position_frenet;
    double yaw;
    double speed;
    int lane;
};

inline double deg2rad(double x)
{
    return x * M_PI / 180;
}

inline double rad2deg(double x)
{
    return x * 180 / M_PI;
}