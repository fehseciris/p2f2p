#pragma once

/**
 * Interface for P2F2P class
 * ^^^^^^^^^^^^^^^^^^^^^^^^^
 * The functionalities that are implemented here in the interface correspond 
 * to the methods used in the Matlab implementation.
 */

#include <iostream>
#include <vector>

#include "util.h"

class Ip2f2p
{
public:
    virtual ~Ip2f2p() = default;

    /**
     * Insert new waypoints 
     */
    virtual void process_points(const std::vector<sPoint>&) = 0;

    /**
     * Convert global states to frenet states
     */
    virtual sFrenet g2f(const sPoint&) = 0;

    /**
     * Convert frenet states to global states
     */
    virtual sPoint f2g(const sFrenet&) = 0;

    /**
     * Calculate path length after interpolation
     */
    virtual double path_length(void) = 0;

    /**
     * Find point on curve at insert distance
     */
    virtual sPoint position(double&) = 0;

    /**
     * Find tangent angle in deg at insert distance on curve
     */
    virtual double tangent_angle_deg(double&) = 0;

    /**
     * Find tangent angle in rad at insert distance on curve
     */
    virtual double tangent_angle_rad(double&) = 0;

    /**
     * Find curvature at insert distance on curve
     */
    virtual double curvature(double&) = 0;

    /**
     * Find change in curvature at insert distance on curve 
     */
    virtual double change_in_curvature(double&) = 0;
};

/* Eof */