#pragma once

#include "util.h"

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
         * To do
         * @param waypoints
         */
        std::vector<sFrenet> g2f(const std::vector<sAPoint>& waypoints);

        /**
         * To do
         * @param frenets
         */
        std::vector<sAPoint> f2g(const std::vector<sFrenet>& frenets);

        /**
         * To do
         * @param waypoints
         */
        static std::vector<sAPoint> expand(std::vector<sPoint> waypoints);

    };

} // namespace p2f2p

/* Eof */