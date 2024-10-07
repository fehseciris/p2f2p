#include "util.h"

std::ostream& operator<<(std::ostream& os, const sPoint& point)
{
    os << "P(" << point.x << "|" << point.y << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<sPoint>& points)
{
    for(auto& it : points)
    {
        os << it << " - ";
    }
    os << "\n";
    return os;
}

std::ostream& operator<<(std::ostream& os, const sFrenet& frenet)
{
    std::string str = {""};
    if(frenet.direction == true)
    {
        str = "left";
    }
    else
    {
        str = "right";
    }
    os << "F{Cartesian" << frenet.cartesian_point 
        << " - Closest" << frenet.closest_point_on_curve
        << " - GeodeticD " << frenet.geodetic_distance
        << " - LateralD " << frenet.lateral_distance
        << " - Direction " << str
        << "}\n";
    return os;
}

/* Eof */