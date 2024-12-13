#pragma once

/**
 * Global project utilities
 * ^^^^^^^^^^^^^^^^^^^^^^^^
 */

#include <iostream>
#include <ostream>
#include <vector>
#include <string>
#include <filesystem>
#include <fstream>

#include "iol.h"

/* Global settings and params */
#define DISCRETIZATION_DISTANCE     0.0001 // Max value 0.0001
#define MAX_NUM_WAYPOINTS           10000
#define MIN_NUM_WAYPOINTS           3
/* Plots */
#define PYTHON_PLOTS                true

/* Global datatypes and operators */
struct sPoint
{
    double x = 0;
    double y = 0;
};
std::ostream& operator<<(std::ostream& os, const sPoint& point);
std::ostream& operator<<(std::ostream& os, const std::vector<sPoint>& points);
bool operator==(const sPoint& a, const sPoint& b);
std::string pto_string(const sPoint& point);

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
bool operator==(const sFrenet& a, const sFrenet& b);

/* Global functions */
static bool file_exists(const std::string& path);
static std::vector<sPoint> collect(int argv, char* argc[]);
void points_to_file(const std::string& filename, 
        const std::vector<sPoint>& points,
        const sPoint& target = {0,0});

/* Eof */