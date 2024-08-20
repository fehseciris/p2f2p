#pragma once

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

std::ostream& operator<<(std::ostream& os, const sAPoint& point)
{
    os << "sAPoint("
       << "x: "         << point.cartesian_point.x << ", "
       << "y: "         << point.cartesian_point.y << ", "
       << "theta: "     << point.theta << ", "
       << "kappa: "     << point.kappa << ", "
       << "dkappa: "    << point.dkappa << ", "
       << "s: "         << point.s
       << ")";
    return os;
}

/* Frenet structure */
struct sFrenet
{
    sPoint cartesian_point;         // Cartesian point
    sPoint closest_point_on_curve;  // Closest point on curve
    double geodetic_distance;       // Geodetic distance
    bool direction;                 // Direction true -> left, false -> right
    double lateral_distance;        // Lateral distance 
};

std::ostream& operator<<(std::ostream& os, const sFrenet& frenet)
{
    os << "sFrenet("
       << "cartesian_point: ("          << frenet.cartesian_point.x << ", " << frenet.cartesian_point.y << "), "
       << "closest_point_on_curve: ("   << frenet.closest_point_on_curve.x << ", " << frenet.closest_point_on_curve.y << "), "
       << "geodetic_distance: "         << frenet.geodetic_distance << ", "
       << "direction: "                 << (frenet.direction ? "left" : "right") << ", "
       << "lateral_distance: "          << frenet.lateral_distance
       << ")";
    return os;
}

/* Eof */