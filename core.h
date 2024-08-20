#pragma once

#include <vector>
#include <cmath>

#include "util.h"
#include "iol.h"

namespace p2f2p
{
    /* Class Core */
    class Core
    {
    public:
        Core();

        ~Core();

        /* Explicit delete */
        Core(const Core&) = delete;
        Core& operator=(const Core&) = delete;
        Core(Core&&) = delete;
        Core& operator=(Core&&) = delete;

        /**
         * Convert global waypoints to Frenet coordinates
         * @param waypoints Input vector of sAPoint (global waypoints)
         * @return std::vector<sFrenet> Vector of Frenet coordinates
         */
        std::vector<sFrenet> g2f(const std::vector<sAPoint>& waypoints);

        /**
         * Convert Frenet coordinates to global waypoints
         * @param frenets Input vector of sFrenet coordinates
         * @return std::vector<sAPoint> Vector of global waypoints (sAPoint)
         */
        std::vector<sAPoint> f2g(const std::vector<sFrenet>& frenets);

        /**
         * Expand sPoint waypoints into sAPoint with default values
         * @param waypoints Input vector of sPoint waypoints
         * @return std::vector<sAPoint> Expanded vector of sAPoint waypoints
         */
        static std::vector<sAPoint> expand(std::vector<sPoint> waypoints);

    };

} // namespace p2f2p

/* Eof */