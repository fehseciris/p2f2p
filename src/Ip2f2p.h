#pragma once

#include <iostream>
#include <vector>

#include "iol.h"
#include "util.h"

class Ip2f2p
{
public:
    virtual ~Ip2f2p() = default;
    virtual void process_points(const std::vector<sPoint>&) = 0;
    virtual sFrenet g2f(const sPoint&) = 0;
    virtual sPoint f2g(const sFrenet&) = 0;
    virtual double path_length(void) = 0;
    virtual sPoint position(double&) = 0;
    virtual double tangent_angle_deg(double&) = 0;
    virtual double tangent_angle_rad(double&) = 0;
    virtual double curvature(double&) = 0;
    virtual double change_in_curvature(double&) = 0;
};