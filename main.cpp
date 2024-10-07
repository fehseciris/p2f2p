#include <iostream>

#include "p2f2p.h"

int main(int argc, char * argv[])
{
    // std::cout << "Built test." << std::endl;

    // std::vector<sPoint> waypoints = {{0,0},{1,1},{2,0},{3,1}};
    std::vector<sPoint> waypoints = {{0,0},{1,1},{0,2},{0,3}};
    // std::vector<sPoint> waypoints = {{0,0},{1,1},{2,5}};
    // std::vector<sPoint> waypoints = {{1,2},{3,5},{4,10}};    

    sPoint point = {3, 2};

    /* Workspace */
    std::shared_ptr<eigen::P2F2P> sptr_p2f2p2 = std::make_shared<eigen::P2F2P>(waypoints);
    sptr_p2f2p2->g2f(point);
    std::cout << *sptr_p2f2p2 << std::endl;




    return EXIT_SUCCESS;
}

/* Eof */