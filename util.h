#pragma once

#include <iostream>
#include <ostream>

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

std::ostream& operator<<(std::ostream& os, const sAPoint& point);

/* Frenet structure */
struct sFrenet
{
    sPoint cartesian_point;         // Cartesian point
    sPoint closest_point_on_curve;  // Closest point on curve
    double geodetic_distance;       // Geodetic distance
    bool direction;                 // Direction true -> left, false -> right
    double lateral_distance;        // Lateral distance 
};

std::ostream& operator<<(std::ostream& os, const sFrenet& frenet);

/* Eof */