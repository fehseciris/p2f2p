#pragma once

#include <iostream>
#include <ostream>
#include <vector>

/* eigen */
#define VERY_LARGE_NUMBER           999999.99f
#define NEGATIVE_ONE                -1
#define ORDER                       2
#define TERMINATION_CRITERIA        0.0001f
#define ALPHA                       0.001f
#define MAXIMUM_ITERATION           25
/* spline */
#define DISCRETIZATION_DISTANCE     0.001
#define MAX_NUM_WAYPOINTS           10000
#define MIN_NUM_WAYPOINTS           3

struct sPoint
{
    double x;
    double y;
};
std::ostream& operator<<(std::ostream& os, const sPoint& point);
std::ostream& operator<<(std::ostream& os, const std::vector<sPoint>& points);

struct sInput
{
    std::vector<sPoint> points;
    sPoint target;
};

struct sCurve
{
    int order;
    std::vector<double> coefficients;
    std::vector<sPoint> points;
};

struct sFrenet
{
    sPoint cartesian_point;
    sPoint closest_point_on_curve;
    double geodetic_distance;
    bool direction; //true -> left; false -> right
    double lateral_distance;
};
std::ostream& operator<<(std::ostream& os, const sFrenet& frenet);

/* Eof */