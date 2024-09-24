#pragma once

#include <vector>
#include <iostream>
#include <variant>
#include <memory>
#include <vector>

#include "spline/src/spline.h"

#include "iol.h"
#include "util.h"

#define DISCRETIZATION_DISTANCE     0.05
#define MAX_NUM_WAYPOINTS           10000

/* Class P2F2P */
class P2F2P
{
public:
    /**
     * Default init without data - init with refresh
     */
    P2F2P();

    /**
     * Init with sPoint data
     * @param input Input vector with sPoint coordinates
     */
    P2F2P(const std::vector<sPoint> &input);

    ~P2F2P();

    /* Explicit delete */
    P2F2P(const P2F2P&) = delete;
    P2F2P& operator=(const P2F2P&) = delete;
    P2F2P(P2F2P&&) = delete;
    P2F2P& operator&(P2F2P&&) = delete;

    /**
     * ### Matlab object functions
     */


    /**
     * ### Customised functions 
     */

    /**
     * Refresh global data
     * @param input Input vector with sPoint coordinates
     * @return void
     */
    void upload(const std::vector<sPoint>& input);

    /**
     * Parse input arguments
     * @param argv Number of arguments
     * @param agc Array of arguments
     * @return Vector of sPoint 
     */
    static std::vector<sPoint> collect(int argv, char* argc[]);

    friend std::ostream& operator<<(std::ostream& os, const P2F2P& o);

private:
    /**
     * Pre calculator for path length and discretisation distance
     * @param input Input vector with sPoint coordinates
     * @return Vector of sPoint
     */
    std::vector<sPoint> pre_calculator(const std::vector<sPoint>& input);

    /* Members_ */
    std::vector<sPoint> points_;

    /* Calculated */
    double path_length_;
    std::vector<double> extract_X_;
    std::vector<double> extract_Y_;

};

std::ostream& operator<<(std::ostream& os, const P2F2P& o);

/* Eof */