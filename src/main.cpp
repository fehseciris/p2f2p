#include <iostream>
#include <fstream>

#include "p2f2p.h"

int main(int argc, char * argv[])
{
    LOG_LEVEL(Level::LDEBUG);
    
    // std::vector<sPoint> waypoints = {{0,0},{1,1},{2,0},{3,1}};
    // std::vector<sPoint> waypoints = {{0,0},{1,1},{0,2},{0,3}};
    // std::vector<sPoint> waypoints = {{0,0},{1,1},{0,2},{0,3},{0,5},{3,6},{3,7},{3,8},{5,2},{8,3},{10,2}};
    // std::vector<sPoint> waypoints = {{0,0},{1,1},{0,2},{0,3},{0,5},{3,6},{3,7},{3,8},{5,2},{8,3},{10,2},{9,1},{2,0},{2,2}};
    // std::vector<sPoint> waypoints = {{0,0},{0,2},{2,4},{4,2},{2,0},{1,0}};
    std::vector<sPoint> waypoints = {{0,0},{0,2},{2,4},{4,2},{2,0},{1,0},{1,1},{3,1},{5,0},{7,2}};
    // std::vector<sPoint> waypoints = {{0,0},{1,1},{2,5}};
    // std::vector<sPoint> waypoints = {{1,2},{3,5},{4,10}};     

    // sPoint point = {3, 2};
    // sPoint point = {5,2};
    // sPoint point = {2,5};
     sPoint point = {3,2};
    // sPoint point = {3,3};
    // sPoint point = {2,2};

    sPoint pdummy;
    sFrenet fdummy;

    /* Spline workspace */
    std::shared_ptr<spline::P2F2P> sptr_spline = std::make_shared<spline::P2F2P>(waypoints);

    /* Kartesian to frenet trafo */
    fdummy = sptr_spline->g2f(point);

    /* Frenet to kartesian trafo */
    pdummy = sptr_spline->f2g(fdummy);

    /* Path length */
    sptr_spline->path_length();

    /* Check values along the path - possible with loop */
    double along_the_path = 5.0;
    sptr_spline->position(along_the_path);
    sptr_spline->tangent_angle_deg(along_the_path);
    sptr_spline->curvature(along_the_path);
    sptr_spline->change_in_curvature(along_the_path);

    /* Useful for debuging */
    // std::cout << pdummy << std::endl;
    // std::cout << fdummy << std::endl;
    // std::cout << *sptr_spline << std::endl;
    
    return EXIT_SUCCESS;
}

/* Eof */