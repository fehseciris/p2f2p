#pragma once

#include <iostream>
#include <fstream>

#include "../eigen-3.4.0/Eigen/QR"
#include "../eigen-3.4.0/Eigen/SVD"
#include "p2f2p.h"

namespace eigen
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
        static void poly_fit_points(const std::vector<sPoint>& points, std::vector<sCoeff>& coeff, int order);
        static sFrenet steepest_gradient_descent(const sCurve& c, const sPoint& target, int init_index, double alpha, int max_iter);

    private:
        /* Cubic */
        static double evaluate(const double x, const double a0, const double a1, const double a2, const double a3);
        static double gradient(const double x, const double p, const double q, const double a0, const double a1, const double a2, const double a3);
        static double residual(const double x, const double p, const double q, const double a0, const double a1, const double a2, const double a3);

        friend class P2F2P;
    };
}; // namespace eigen

namespace spline
{
    class Compute : public eigen::Compute
    {
    public:
        ~Compute() = default;
        /* Explicit delete */
        Compute(const Compute&) = delete;
        Compute& operator=(const Compute&) = delete;
        Compute(Compute&&) = delete;
        Compute& operator=(Compute&&) = delete;
        
        static double angle2x(const sPoint& a, const sPoint& b);
        static double angle2y(const sPoint& a, const sPoint& b);

        friend class P2F2P;
    };
}; // namespace spline

/* Eof */