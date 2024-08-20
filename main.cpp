#include <iostream>

#include "p2f2p.h"

int main(int argc, char * argv[])
{
    // std::cout << "Built test." << std::endl;

    /* Init with arguments */
    // std::shared_ptr<P2F2P> sptr_p2f2p = std::make_shared<P2F2P>(P2F2P::collect(argc, argv));

    /* Init with point vector */
    std::vector<sPoint> waypoints = {{0, 0}, {1, 1}, {2, 0}, {3, 1}};
    std::shared_ptr<P2F2P> sptr_p2f2p = std::make_shared<P2F2P>(waypoints);
    sptr_p2f2p->global2frenet();
    std::vector<sFrenet> frenets = sptr_p2f2p->get_frenets();

    /**
     * Use frenet object functionality ...
     */

    return EXIT_SUCCESS;
}

/* Eof */