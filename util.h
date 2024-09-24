#pragma once

#include <iostream>
#include <ostream>
#include <set>

/* Defines */
#define DUMMY                   1

/* Point structure */
struct sPoint
{
    double x;                       // x-Position global
    double y;                       // y-Position global 
};

std::ostream& operator<<(std::ostream& os, const sPoint& point);

std::ostream& operator<<(std::ostream& os, const std::set<sPoint>& points);

/* Eof */