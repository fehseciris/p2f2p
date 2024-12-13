#include "num.h"

/**
 * Calculate the Euclidean distance between two points A and B.
 * @param A The starting point of the distance calculation.
 * @param B The ending point of the distance calculation.
 * @return The Euclidean distance between points A and B.
 */
double spline::Compute::eucledian_distance(const sPoint A, const sPoint B)
{
    double diffY = B.y - A.y;
    double diffX = B.x - A.x;
    return sqrt(diffY * diffY + diffX * diffX);
}

/**
 * Calculate the 2D cross product between points A and B.
 * The cross product in 2D results in a scalar that represents the orientation between two vectors.
 * @param A The first point.
 * @param B The second point.
 * @return The scalar value of the cross product (A × B).
 */
double spline::Compute::cross_product(const sPoint A, const sPoint B)
{
    return ((A.x * B.y) - (A.y * B.x));
}

/**
 * Calculate the 2D cross product between the vector Ga-Gb and the vector Ga-P.
 * This function helps determine the position of point P relative to the vector Ga-Gb.
 * @param Ga The starting point of the vector.
 * @param Gb The ending point of the vector.
 * @param P The point whose position relative to the vector Ga-Gb is being calculated.
 * @return A scalar representing the cross product of the two vectors.
 *         A positive result means P lies to the left of Ga-Gb, while a negative result means P lies to the right.
 */
double spline::Compute::cross_product(const sPoint Ga, const sPoint Gb, const sPoint P)
{
    return ((Gb.x - Ga.x) * (P.y - Ga.y) - (Gb.y - Ga.y) * (P.x - Ga.x));
}

/**
 * Based on to points and a generated vector the angle to x-axis will be
 * calculated and returned.
 * @param a End point vector
 * @param b Begin point vector
 * @return angle to x-axis
 */
double spline::Compute::angle2x(const sPoint& a, const sPoint& b) 
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double angle_radians = std::atan2(dy, dx);  // atan2 gibt den Winkel in Radianten zurück
    double angle_degrees = angle_radians * (180.0 / M_PI);
    return angle_degrees;
}

/**
 * Based on to points and a gernerated vector the angle to y-axis will be 
 * calculated and returned.
 * @param a End point vector
 * @param b Begin point vector
 * @return angle to y-axis
 */
double spline::Compute::angle2y(const sPoint& a, const sPoint& b)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double angle_radians = std::atan2(dx, dy);
    double angle_degrees = angle_radians * (180.0 / M_PI);
    std::cout << "-> this is inside: " << angle_degrees << std::endl;
    return angle_degrees;
}

/**
 * Evaluate a polynomial of degree 3 at a given x-coordinate.
 * The polynomial form is: f(x) = a3*x^3 + a2*x^2 + a1*x + a0.
 * @param x The point at which the polynomial is evaluated.
 * @param a0 The constant term of the polynomial.
 * @param a1 The coefficient of the linear term.
 * @param a2 The coefficient of the quadratic term.
 * @param a3 The coefficient of the cubic term (default is 0).
 * @return The polynomial value at x.
 */
double spline::Compute::evaluate(const double x, const double a0, const double a1, const double a2, const double a3 = 0)
{
    return ((a3 * x * x * x) + (a2 * x * x) + (a1 * x) + a0);
}

/**
 * Calculate the gradient of a quadratic error term that represents the distance between a point (p, q) and a polynomial.
 * The gradient provides the direction and rate of change of the error with respect to x.
 * @param x The x-coordinate at which the gradient is calculated.
 * @param p The x-coordinate of the point to be tested.
 * @param q The y-coordinate of the point to be tested.
 * @param a0 The constant term of the polynomial.
 * @param a1 The coefficient of the linear term of the polynomial.
 * @param a2 The coefficient of the quadratic term of the polynomial.
 * @param a3 The coefficient of the cubic term of the polynomial (default is 0).
 * @return The gradient of the quadratic error term with respect to x.
 */
double spline::Compute::gradient(const double x, const double p, const double q, const double a0, const double a1, const double a2, const double a3 = 0)
{
    double grad_f_x = 3 * a3 * x * x + 2 * a2 * x + a1;
    double df_dx = 2 * (x - p) + 2 * (spline::Compute::evaluate(x, a0, a1, a2, a3) - q) * grad_f_x;
    return df_dx;
}

/**
 * Calculate the residual error, representing the Euclidean distance between a point (p, q) and a point on the polynomial.
 * This function evaluates the difference between the point and the polynomial value at x.
 * @param x The x-coordinate of the point on the polynomial.
 * @param p The x-coordinate of the point to be tested.
 * @param q The y-coordinate of the point to be tested.
 * @param a0 The constant term of the polynomial.
 * @param a1 The coefficient of the linear term of the polynomial.
 * @param a2 The coefficient of the quadratic term of the polynomial.
 * @param a3 The coefficient of the cubic term of the polynomial (default is 0).
 * @return The residual error, representing the Euclidean distance between the point (p, q) and the polynomial point (x, f(x)).
 */
double spline::Compute::residual(const double x, const double p, const double q, const double a0, const double a1, const double a2, const double a3 = 0)
{
    double diffQ = q - spline::Compute::evaluate(x, a0, a1, a2, a3); 
    double diffP = p - x;
    return sqrt(diffQ * diffQ + diffP * diffP);
}

/* Eof */