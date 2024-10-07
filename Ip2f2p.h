#pragma once

#include <iostream>
#include <vector>

#include "iol.h"
#include "util.h"

class Ip2f2p
{
public:
    virtual ~Ip2f2p();
    virtual void process_points(const std::vector<sPoint>&) = 0;
    virtual sFrenet g2f(const sPoint&) = 0;
    virtual sPoint f2g(const sFrenet&) = 0;

    // virtual double path_length(void) = 0;
    // virtual void position(void) = 0;
    // virtual void tangent_angle(void) = 0;
    // virtual void curvatur(void) = 0;
    // virtual void changeln_curvature(void) = 0;
};