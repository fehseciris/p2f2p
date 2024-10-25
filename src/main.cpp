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
    // std::vector<sPoint> waypoints = {{0,0},{0,2},{2,4},{4,2},{2,0},{1,0}};
    std::vector<sPoint> waypoints = {{0,0},{0,2},{2,4},{4,2},{2,0},{1,0},{1,1},{3,1},{5,0},{7,2}};
    // std::vector<sPoint> waypoints = {{0,0},{1,1},{2,5}};
    // std::vector<sPoint> waypoints = {{1,2},{3,5},{4,10}};     

    // sPoint point = {3, 2};
    sPoint point = {5,2};
    // sPoint point = {2,5};
    // sPoint point = {3,2};
    // sPoint point = {3,3};
    // sPoint point = {2,2};

    sPoint dummy;
    sFrenet frenet;

    /* Open file to write waypoints and the target point */
    std::ofstream file("../../plot/waypoints.txt");
    if (file.is_open()) 
    {
        for (const auto& point : waypoints) 
        {
            file << point.x << "," << point.y << std::endl;  
        }
        file << "target," << point.x << "," << point.y << std::endl;
        file.close();
        LOG(Level::LINFO, "Waypoints and target point successfully written to waypoints.txt");
        if(PLOTS_ACTIVE_WAYPTS)
        {
            system("start python ../../plot/waypoints.py"); 
        }
    } 
    else
    {
        LOG(Level::LERROR, "Unable to open file ../../plot/waypoints.txt for writing.");
    }
    if(PLOTS_ACTIVE_WAYPTS == false)
    {
        LOG(Level::LWARNING, "Plot waypoints disabled.");
    }
    
    /* Eigen workspace - doesnt work fine - numerical instability */
    // std::shared_ptr<eigen::P2F2P> sptr_p2f2p = std::make_shared<eigen::P2F2P>(waypoints);
    // frenet = sptr_p2f2p->g2f(point);
    // std::cout << frenet << std::endl;

    /* Spline workspace */
    // std::shared_ptr<spline::P2F2P> sptr_spline = std::make_shared<spline::P2F2P>(waypoints);
    std::shared_ptr<spline::P2F2P> sptr_spline = std::make_shared<spline::P2F2P>();
    sptr_spline->process_points(waypoints);
    frenet = sptr_spline->g2f(point);
    // std::cout << frenet << std::endl;
    dummy = sptr_spline->f2g(frenet);
    // std::cout << dummy << std::endl;
    // std::cout << *sptr_spline << std::endl;
    sptr_spline->path_length();
    for(double along_the_path = 0; along_the_path < sptr_spline->path_length(); along_the_path++)
    {
        sptr_spline->position(along_the_path);
        sptr_spline->tangent_angle_deg(along_the_path);
        sptr_spline->curvature(along_the_path);
        sptr_spline->change_in_curvature(along_the_path);
    }
    
    return EXIT_SUCCESS;
}

/* Eof */