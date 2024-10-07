#include "util.h"

std::ostream& operator<<(std::ostream& os, const sPoint& point)
{
    os << "Point("
       << "x: "         << point.x << ", "
       << "y: "         << point.y << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<sPoint>& points)
{
    for(auto& it : points)
    {
        os << it << " | ";
    }
    return os;
}

/* Eof */