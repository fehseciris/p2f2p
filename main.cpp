#include <iostream>
#include <fstream>

#include "p2f2p.h"

int main(int argc, char * argv[])
{
    // std::cout << "Built test." << std::endl;

    // std::vector<sPoint> waypoints = {{0,0},{1,1},{2,0},{3,1}};
    // std::vector<sPoint> waypoints = {{0,0},{1,1},{0,2},{0,3}};
    // std::vector<sPoint> waypoints = {{0,0},{1,1},{0,2},{0,3},{0,5},{3,6},{3,7},{3,8},{5,2},{8,3},{10,2}};
    // std::vector<sPoint> waypoints = {{0,0},{1,1},{0,2},{0,3},{0,5},{3,6},{3,7},{3,8},{5,2},{8,3},{10,2},{9,1},{2,0},{2,2}};
    std::vector<sPoint> waypoints = {{0,0},{0,2},{2,4},{4,2},{2,0},{1,0}};
    // std::vector<sPoint> waypoints = {{0,0},{1,1},{2,5}};
    // std::vector<sPoint> waypoints = {{1,2},{3,5},{4,10}};     

    // sPoint point = {3, 2};
    sPoint point = {5,2};
    // sPoint point = {2,5};
    // sPoint point = {3,2};
    // sPoint point = {3,3};
    // sPoint point = {2,2};

    sPoint dummy = {0, 0};
    sFrenet frenet;

    /* Open file to write waypoints and the target point */
    std::ofstream file("../plot/waypoints.txt");
    if (file.is_open() && PLOTS_ACTIVE) 
    {
        for (const auto& point : waypoints) 
        {
            file << point.x << "," << point.y << std::endl;  
        }
        file << "target," << point.x << "," << point.y << std::endl;
        file.close();
        LOG(Level::LINFO, "Waypoints and target point successfully written to waypoints.txt");
        system("start python ../plot/waypoints.py"); 
    } 
    else if (PLOTS_ACTIVE == false)
    {
        LOG(Level::LWARNING, "Write in files disabled.");
    }
    else
    {
        LOG(Level::LERROR, "Unable to open file for writing.");
    }
    
    /* Eigen workspace */
    // std::shared_ptr<eigen::P2F2P> sptr_p2f2p = std::make_shared<eigen::P2F2P>(waypoints);
    // frenet = sptr_p2f2p->g2f(point);
    // std::cout << frenet << std::endl;

    /* Spline workspace */
    std::shared_ptr<spline::P2F2P> sptr_spline = std::make_shared<spline::P2F2P>(waypoints);
    sptr_spline->process_points(waypoints);
    frenet = sptr_spline->g2f(point);
    // std::cout << frenet << std::endl;
    dummy = sptr_spline->f2g(frenet);
    // std::cout << dummy << std::endl;
    // std::cout << *sptr_spline << std::endl;
    sptr_spline->path_length();
    for(double i = 1.; i < sptr_spline->path_length(); i++)
    {
        sptr_spline->position(i);
        sptr_spline->tangent_angle(i);
    }
    
    return EXIT_SUCCESS;
}

/* Eof */