#include <iostream>

#include "p2f2p.h"

int main(int argc, char * argv[])
{
    // std::cout << "Built test." << std::endl;
    
    /**
     * Use frenet object functionality ...
     */

    /* Data */
    // std::vector<sPoint> waypoints = {{0,0},{1,1},{2,0},{3,1}};
    std::vector<sPoint> waypoints = {{0,0},{1,1},{2,5}};
    // std::vector<sPoint> new_waypoints = {{1,2},{3,5},{4,10}};

    /* 1. Empty init */
    std::shared_ptr<P2F2P> sptr_p2f2p1 = std::make_shared<P2F2P>();
    sptr_p2f2p1->upload(waypoints);
    std::cout << *sptr_p2f2p1;


    

    /* 2. Init with point vector */
    std::shared_ptr<P2F2P> sptr_p2f2p2 = std::make_shared<P2F2P>(waypoints);
    std::cout << *sptr_p2f2p2;


    /* Refresh with new waypoints */
    // sptr_p2f2p3->upload(new_waypoints);

    return EXIT_SUCCESS;
}

/* Eof */