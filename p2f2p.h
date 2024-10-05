#pragma once

#include "Util.h"
#include "NumericalMethods.h"

class P2F2P
{
public:
    P2F2P(const std::vector<sPoint>& points);
    ~P2F2P();
    void process_points(const std::vector<sPoint>& points);
    sFrenet g2f(const sPoint& target);

private:
    void clear(void);


    curve obtain_curve(const vector<point> &points);
    int find_nearest_point(const curve &c, const point &target);
    frenet distance_to_curve(const curve &c, point &target, int min_index);
    void geodetic_distance(const curve &c, frenet &frame, int min_index);

    std::vector<sPoint> waypoints_;
    sCurve curve_;
};

/* Eof */