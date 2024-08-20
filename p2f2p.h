#pragma once

#include <vector>
#include <iostream>
#include <variant>
#include <memory>

#include "iol.h"
#include "util.h"
#include "core.h"

/* Class P2F2P */
class P2F2P
{
private:
    /* Members_ */
    std::vector<sAPoint> points_;
    std::vector<sFrenet> frenets_;

public:
    using VariantVector = std::variant<std::vector<sPoint>, std::vector<sFrenet>>;

    /**
     * Default init without data - init with refresh
     */
    P2F2P();

    /**
     * Init with sFrenet data
     * @param input Input vector with sFrenet coordinates
     */
    P2F2P(std::vector<sFrenet> input);

    /**
     * Init with sPoint data
     * @param input Input vector with sPoint coordinates
     */
    P2F2P(std::vector<sPoint> input);

    /**
     * Init with collect from parser
     * @param input Vector with sFrenet or sPoint data
     */
    P2F2P(VariantVector input);

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
     * Find closest point on reference path to global point
     * @return void
     */
    void closest_point();

    /**
     * Projects sequence of points onto path
     * @return void
     */
    void closest_points_to_sequence();

    /**
     * Find orthogonal projections between path tangent vector and query point
     * @return void
     */
    void closest_projections();

    /**
     * Return curvature at arclength
     * @param s The arclength along the path
     * @return double The curvature at the given arclength
     */
    double curvature(double s);

    /**
     * Calculate change in curvature along the path
     * @return void
     */
    void change_in_curvature();

    /**
     * Convert Frenet states to global states
     * @return void
     */
    void frenet2global();

    /**
     * Convert global states to Frenet states
     * @return void
     */
    void global2frenet();

    /**
     * Interpolate reference path at provided arc lengths
     * @return void
     */
    void interpolate();

    /**
     * Return xy-position at arclength
     * @param s The arclength along the path
     * @return sPoint The position (x, y) at the given arclength
     */
    sPoint position(double s);

    /**
     * Return tangent angle at arclength
     * @param s The arclength along the path
     * @return double The tangent angle at the given arclength
     */
    double tangentAngle(double s);

    /**
     * Display reference path in figure
     * @return void
     */
    void show();

    /**
     * Copy reference path
     * @return void
     */
    void copy();

    /**
     * ### Customised functions 
     */

    /**
     * Return the list of sAPoints
     * @return std::vector<sAPoint> The list of global points
     */
    std::vector<sAPoint> get_points();

    /**
     * Return the list of sFrenet coordinates
     * @return std::vector<sFrenet> The list of Frenet coordinates
     */
    std::vector<sFrenet> get_frenets();

    /**
     * Return the total path length
     * @return double The total length of the path
     */
    double path_length();

    /**
     * Refresh Frenet data
     * @param input Input vector with sFrenet coordinates
     * @return void
     */
    void upload(std::vector<sFrenet> input);

    /**
     * Refresh global data
     * @param input Input vector with sPoint coordinates
     * @return void
     */
    void upload(std::vector<sPoint> input);

    /**
     * Parse input arguments
     * @param argv Number of arguments
     * @param argc Array of arguments
     * @return VariantVector A vector of sPoint or sFrenet depending on input
     */
    static VariantVector collect(int argv, char* argc[]);

};

/* Eof */