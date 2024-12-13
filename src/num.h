#pragma once

/**
 * Compute class
 * ^^^^^^^^^^^^^
 * This class implements the numerical computations for the transformation.
 */

#include <iostream>
#include <fstream>

#include "p2f2p.h"

namespace spline
{
    class Compute
    {
    public:
        ~Compute() = default;
        /* Explicit delete */
        Compute(const Compute&) = delete;
        Compute& operator=(const Compute&) = delete;
        Compute(Compute&&) = delete;
        Compute& operator=(Compute&&) = delete;
        
        static double eucledian_distance(const sPoint A, const sPoint B);
        static double cross_product(const sPoint A, const sPoint B);
        static double cross_product(const sPoint Ga, const sPoint Gb, const sPoint P);
        static double angle2x(const sPoint& a, const sPoint& b);
        static double angle2y(const sPoint& a, const sPoint& b);

    private:
        static double evaluate(const double x, const double a0, const double a1, const double a2, const double a3);
        static double gradient(const double x, const double p, const double q, const double a0, const double a1, const double a2, const double a3);
        static double residual(const double x, const double p, const double q, const double a0, const double a1, const double a2, const double a3);

        /* Access to private methods for P2F2P maybe useful in future */
        friend class P2F2P;
    };

}; // namespace spline

/* Eof */