#pragma once

#include "Eigen-3.3/Eigen/Core"
#include "spdlog/spdlog.h"

#include <cmath>
#include <vector>

#define LOGGER spdlog::get("console")

using Point = Eigen::Vector2d;
using Obstacles = std::vector<std::vector<double>>;

struct Path : public std::vector<Point>
{
    Path() : std::vector<Point>(0)
    {
    }

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

struct FrenetMapPoint
{
    FrenetMapPoint() : FrenetMapPoint(0, Point::Zero()){};
    FrenetMapPoint(double s, Point const &d) : s(s), dVector(d){};
    double s;
    Point dVector;
};

struct FrenetPoint
{
    FrenetPoint() : FrenetPoint(0, 0){};
    FrenetPoint(double s, double d) : s(s), d(d){};
    double s;
    double d;
};

inline double deg2rad(double x)
{
    return x * M_PI / 180;
}

inline double rad2deg(double x)
{
    return x * 180 / M_PI;
}