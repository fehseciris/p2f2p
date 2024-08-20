#pragma once

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
    P2F2P(/* Default init */);
  
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
     * Find closest point on refernce path to gloabel point
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
     * @return void 
     */
    void curvature();

    /**
     * Return change-in-curvature at arclength
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
     * @return void 
     */
    void position();

    /**
     * Return tangent angle at arclength
     * @return void 
     */
    void tangentAngle();

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
     * To do
     */
    std::vector<sAPoint> get_points();

    /**
     * To do
     */
    std::vector<sFrenet> get_frenets();

    /**
     * Refresh frenet data 
     * 
     * @param input Input vector with sFrenet coordinates 
     * @return void
     */
    void upload(std::vector<sFrenet> input);

    /**
     * Refresh global data
     * 
     * @param input Input vector with sPoint coordinates
     * @return void 
     */
    void upload(std::vector<sPoint> input);

    /**
     * Parse input arguments 
     * 
     * @param argv amount of arguments 
     * @param argc arguments
     * @return Vector sPoint/sFrenet depeding on input 
     */
    static VariantVector collect(int argv, char* argc[]);

};

/* Eof */