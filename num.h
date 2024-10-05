#pragma once

#include <iostream>

#include <Eigen/QR>
#include "p2f2p.h"

class NumMethods
{
public:
    double eucledian_distance(const sPoint A, const sPoint B);
    double cross_product(const sPoint A, const sPoint B);
    void poly_fit_points(const std::vector<sPoint>& points, std::vector<double>& coeff, int order);
    sFrenet steepest_gradient_descent(const sCurve& c, sPoint& target, int init_index, double alpha, int max_iter);

private:
    double evaluate(const double x, const double a1, const double a2, const double a3);
    double gradient(const double x, const double p, const double q, const double a1, const double a2, const double a3);
    double residual(const double x, const double p, const double q, const double a1, const double a2, const double a3);

    friend class P2F2P;
};

/* Eof */