#include "num.h"

double NumMethods::eucledian_distance(const sPoint A, const sPoint B)
{
    return sqrt(pow((B.y - A.y),2) + pow((B.x - A.x),2));
}

double NumMethods::cross_product(const sPoint A, const sPoint B)
{
    return ((A.x*B.y) - (A.y*B.x));
}

double NumMethods::evaluate(const double x, const double a1, const double a2, const double a3)
{
    return ((a1*x*x) + (a2*x) + a3);
}

double NumMethods::gradient(const double x, const double p, const double q, const double a1, const double a2, const double a3)
{
    return (-(4*pow(a1,3)*pow(x,3)) - (6*a1*a2*pow(x,2)) - (2*x*(pow(a2,2) - 1 + (2*a1*(a3-q)))) + (-2*p - 2*a2*a3 - 2*q*a2));
}

double NumMethods::residual(const double x, const double p, const double q, const double a1, const double a2, const double a3)
{
    return (pow((q - evaluate(x, a1, a2, a3)),2) + pow((p - x),2)) ;
}

/**
 * Fitting a polynomial to the input points (uses the Eigen library)
 *
 * @param points Input waypoints
 * @param coeff Output coefficients of the polynomial equation
 * @param order Required order of the polynomial
 * @return void
 */
void NumMethods::poly_fit_points(const std::vector<sPoint>& points, std::vector<double>& coeff, int order)
{
    Eigen::MatrixXd A(points.size(), order+1);
    Eigen::VectorXd xv(points.size());
    Eigen::VectorXd yv(points.size());
    Eigen::VectorXd result;
    for(int i = 0; i < points.size(); i++)
    {
        xv(i) = points[i].x;
        yv(i) = points[i].y;
    }
    for (size_t i = 0; i < points.size(); i++)
    {
        for (size_t j = 0; j < order+1; j++)
        {
            A(i, j) = pow(points[i].x, j);
        }
    }
    result = A.householderQr().solve(yv);
    coeff.resize(order+1);
    for (size_t i = 0; i < order+1; i++)
    {
        coeff[i] = result[i];
    }
}

/**
 * Steepest gradient descent for estimating the closest point from the target on the continuous curve defined by the 2nd order polynomial equation.
 * This works only for second order polynomial, in the current format.
 *
 * @param curve Equation describing the curve
 * @param target Target/input point
 * @param closest Waypoint to the target, serves as the initial estimate
 * @param alpha Damping parameter
 * @param max_iter Termination criterion
 * @return frenet Frenet frame describing the target
 */
sFrenet NumMethods::steepest_gradient_descent(const sCurve& curve, const sPoint& target, int init_index, double alpha, int max_iter)
{
    double x = curve.points[init_index].x;  double y = curve.points[init_index].y;
    double p = target.x;  double q = target.y;
    double prev_residual = VERY_LARGE_NUMBER;
    double residual_;

    sPoint closest_approach;
    sFrenet frame;
    frame.cartesian_point = target;

    for(int i = 0; i<max_iter; i++)
    {
        x = x - (alpha*gradient(x, p, q, curve.coefficients[2], curve.coefficients[1], curve.coefficients[0])); //gradient estimation and update
        residual_ = residual(x, p, q, curve.coefficients[2], curve.coefficients[1], curve.coefficients[0]);  //new residual computation
        if(fabs(residual_ - prev_residual) < TERMINATION_CRITERIA)        //checking termination criterion
        {
            closest_approach.x = x;
            closest_approach.y = y;
            frame.closest_point_on_curve = closest_approach;
            frame.lateral_distance = residual_;

            return frame;
        }
        else
        {
            prev_residual = residual_;
        }
    }
    //in case the maximum number of iterations have been exceeded without convergence (it starts getting weird at this point), the last updated values are returned
    closest_approach.x = x;
    closest_approach.y = y;
    frame.closest_point_on_curve = closest_approach;
    frame.lateral_distance = residual_;

    return frame;
}

/* Eof */