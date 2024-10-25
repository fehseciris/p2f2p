#pragma once

#include <iostream>
#include <ostream>
#include <vector>
#include <string>
#include <filesystem>

/* Parameters for eigen */
#define MIN_WAYPOINTS               10
#define NEGATIVE_ONE                -1
#define ORDER                       2
#define TERMINATION_CRITERIA        0.0001f
#define ALPHA                       0.1f
#define MAXIMUM_ITERATION           25
#define MAX_GRADIENT                1e10
#define MAX_RESIDUAL                1e200
/* Parameters for spline */
#define DISCRETIZATION_DISTANCE     0.0001 // Max value 0.0001
#define MAX_NUM_WAYPOINTS           10000
#define MIN_NUM_WAYPOINTS           3
/* Plots */
#define PLOTS_ACTIVE_POINTS         false
#define PLOTS_ACTIVE_SPLINE         true
#define PLOTS_ACTIVE_WAYPTS         false

/* Global coordinates */
struct sPoint
{
    double x = 0;
    double y = 0;
};
std::ostream& operator<<(std::ostream& os, const sPoint& point);
std::ostream& operator<<(std::ostream& os, const std::vector<sPoint>& points);
std::string to_string(const sPoint& point);

struct sCoeff 
{
    double a0 = 0; // Constant term
    double a1 = 0; // Linear coefficient
    double a2 = 0; // Quadratic coefficient
    double a3 = 0; // Cubic coefficient
};
std::ostream& operator<<(std::ostream& os, const sCoeff& coeff);

struct sCurve
{
    int order;
    std::vector<sCoeff> coefficients;
    std::vector<sPoint> points;
};

/* Frenet coordinates */
struct sFrenet
{
    sPoint cartesian_point = {0,0};
    sPoint closest_cartesian_point = {0,0};
    sPoint closest_point_on_curve = {0,0};
    double geodetic_distance = 0;
    bool direction = true; //true -> left; false -> right
    double lateral_distance = 0;
};
std::ostream& operator<<(std::ostream& os, const sFrenet& frenet);

bool file_exists(const std::string& path);

/* Eof */