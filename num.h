#pragma once

#include <iostream>

#include "eigen-3.4.0/Eigen/QR"
#include "p2f2p.h"

class NumMethods
{
public:
    static double eucledian_distance(const sPoint A, const sPoint B);
    static double cross_product(const sPoint A, const sPoint B);
    static void poly_fit_points(const std::vector<sPoint>& points, std::vector<double>& coeff, int order);
    static sFrenet steepest_gradient_descent(const sCurve& c, const sPoint& target, int init_index, double alpha, int max_iter);

private:
    static double evaluate(const double x, const double a1, const double a2, const double a3);
    static double gradient(const double x, const double p, const double q, const double a1, const double a2, const double a3);
    static double residual(const double x, const double p, const double q, const double a1, const double a2, const double a3);

    friend class P2F2P;
};

/* Eof */