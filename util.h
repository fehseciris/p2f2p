#pragma once

/* Libs */
#include <list>
#include <iostream>
#include <sstream>
#include <vector>
#include <cstring>
#include <algorithm>
#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <iomanip>
#include <exception>
#include <functional>
#include <variant>
#include <chrono>
#include <string>
#include <ctime>
#include <memory>

#include "iol.h"

/* Defines */
#define DUMMY                   1

/* Point structure */
struct sPoint
{
    double x;                       // x-Position global
    double y;                       // y-Position global 
};

/* Advanced Point structure */
struct sAPoint
{
    sPoint cartesian_point;         // Cartesian point
    double theta;                   // Orientation global (rad)
    double kappa;                   // Curvature curve
    double dkappa;                  // Derivative curvature
    double s;                       // Arc length along the path
};

/* Frenet structure */
struct sFrenet
{
    sPoint cartesian_point;         // Cartesian point
    sPoint closest_point_on_curve;  // Closest point on curve
    double geodetic_distance;       // Geodetic distance
    bool direction;                 // Direction true -> left, false -> right
    double lateral_distance;        // Lateral distance 
};

/* Eof */