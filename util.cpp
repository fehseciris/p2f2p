#include "util.h"

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