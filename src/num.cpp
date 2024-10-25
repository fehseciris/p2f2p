#include "num.h"

/**
 * Begin eigen eigen eigen eigen eigen eigen eigen eigen eigen eigen eigen eigen eigen eigen eigen 
 */

/**
 * Calculate the Euclidean distance between two points A and B.
 * @param A The starting point of the distance calculation.
 * @param B The ending point of the distance calculation.
 * @return The Euclidean distance between points A and B.
 */
double eigen::Compute::eucledian_distance(const sPoint A, const sPoint B)
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
double eigen::Compute::cross_product(const sPoint A, const sPoint B)
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
double eigen::Compute::cross_product(const sPoint Ga, const sPoint Gb, const sPoint P)
{
    return ((Gb.x - Ga.x) * (P.y - Ga.y) - (Gb.y - Ga.y) * (P.x - Ga.x));
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
double eigen::Compute::evaluate(const double x, const double a0, const double a1, const double a2, const double a3 = 0)
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
double eigen::Compute::gradient(const double x, const double p, const double q, const double a0, const double a1, const double a2, const double a3 = 0)
{
    double grad_f_x = 3 * a3 * x * x + 2 * a2 * x + a1;
    double df_dx = 2 * (x - p) + 2 * (eigen::Compute::evaluate(x, a0, a1, a2, a3) - q) * grad_f_x;
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
double eigen::Compute::residual(const double x, const double p, const double q, const double a0, const double a1, const double a2, const double a3 = 0)
{
    double diffQ = q - eigen::Compute::evaluate(x, a0, a1, a2, a3); 
    double diffP = p - x;
    return sqrt(diffQ * diffQ + diffP * diffP);
}

/**
 * Fitting a polynomial to the input points (uses the Eigen library)
 * @param points Input waypoints
 * @param coeff Output coefficients of the polynomial equation
 * @param order Required order of the polynomial
 * @return void
 */
void eigen::Compute::poly_fit_points(const std::vector<sPoint>& points, std::vector<sCoeff>& coeffs, int order)
{
    std::ofstream file("../../plot/coefficients.txt");
    if (!file.is_open()) 
    {
        LOG(Level::LERROR, "Unable to open file ../../plot/coefficients.txt for writing.");
        return;
    }
    for (int i = order - 1; i < points.size() - (size_t)(order - 1); ++i) 
    {
        /* Select 5 points (2 previous, 1 current, 2 next) */
        std::vector<sPoint> sub_points(points.begin() + i - (order - 1), points.begin() + i + (order - 1));
        /* Find the minimum and maximum x-values (without normalizing) */
        double x_min = std::numeric_limits<double>::min();
        double x_max = std::numeric_limits<double>::max();
        for (const auto& point : sub_points) 
        {
            if (point.x < x_min) x_min = point.x;
            if (point.x > x_max) x_max = point.x;
        }
        double x_range = x_max - x_min;
        /* Avoid division by zero */
        if (x_range == 0) x_range = 1.0;
        /* Define matrices for polynomial fitting */
        Eigen::MatrixXd A(sub_points.size(), order + 1);
        Eigen::VectorXd yv(sub_points.size());
        Eigen::VectorXd result;
        /* Fill the matrix A and the vector yv */
        for (int j = 0; j < sub_points.size(); j++) 
        {
            double x = sub_points[j].x;
            double y = sub_points[j].y;
            yv(j) = y;
            /* Use direct multiplication for x^j */
            double x_power = 1.0;
            for (int k = 0; k <= order; k++) 
            {
                A(j, k) = x_power;
                x_power *= x;
            }
        }
        /* Use Singular Value Decomposition (SVD) for a more stable solution */
        result = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(yv);
        /* Store the coefficients in sCoeff structure */
        sCoeff coeff;
        for (int k = 0; k <= order; k++) 
        {
            if (k == 0) 
                coeff.a0 = result[k];
            if (k == 1) 
                coeff.a1 = result[k];
            if (k == 2) 
                coeff.a2 = result[k];
            if (k == 3) 
                coeff.a3 = result[k];
        }
        coeffs.push_back(coeff);
        /* Write the coefficients to the file */
        file << coeff.a0 << "," << coeff.a1 << "," << coeff.a2 << "," << coeff.a3 << std::endl;
    }
    file.close();
    /* Python-Skript ausführen */
    system("start python ../../plot/coefficients.py"); 
}

/**
 * Steepest gradient descent for estimating the closest point from the target on the continuous curve defined by the 2nd order polynomial equation.
 * This works only for second order polynomial, in the current format.
 * @param curve Equation describing the curve
 * @param target Target/input point
 * @param closest Waypoint to the target, serves as the initial estimate
 * @param alpha Damping parameter
 * @param max_iter Termination criterion
 * @return frenet Frenet frame describing the target
 */
sFrenet eigen::Compute::steepest_gradient_descent(const sCurve& curve, const sPoint& target, int init_index, double alpha, int max_iter)
{
    /* Initial values */
    double x = curve.points[init_index].x;  
    double y = curve.points[init_index].y;
    double p = target.x;
    double q = target.y;
    double prev_residual = std::numeric_limits<double>::max();
    double residual_num = 0;
    sPoint closest_approach;  
    sFrenet frame;  
    frame.cartesian_point = target;  
    /* Ensure the index is valid (if init_index is too small, default to 2) */
    if (init_index < 2) 
    {
        init_index = 2;
    }
    /* Retrieve the polynomial coefficients for the current segment */
    const sCoeff& coeff = curve.coefficients[init_index];
    std::cout << "used " << coeff;
    /* Iterative optimization (gradient descent) */
    for (int i = 0; i < max_iter; i++) 
    {
        std::cout << "x: " << x
                << " | p: " << p 
                << " | q: " << q
                << std::endl;
        /* Calculate the gradient using the polynomial's coefficients (a1, a2, a3) */
        double grad = eigen::Compute::gradient(x, p, q, coeff.a0, coeff.a1, coeff.a2, coeff.a3);
        std::cout << "grad: " << grad;
        /* If the gradient is NaN or exceeds a defined threshold, abort */
        if (std::isnan(grad) || fabs(grad) > MAX_GRADIENT) 
        {
            LOG(Level::LERROR, "Gradient out of bounds or NaN detected, aborting.");
            throw std::runtime_error("Gradient out of bounds or NaN detected");
        }
        /* Update the x value using the gradient with a constant learning rate (alpha) */
        x = x - (alpha * grad);
        /* Calculate the new residual (distance between the new point and the target point) */
        residual_num = eigen::Compute::residual(x, p, q, coeff.a0, coeff.a1, coeff.a2, coeff.a3);
        std::cout << " | residual: " << residual_num << std::endl;
        /* If the residual is NaN or exceeds a defined threshold, abort */
        if (std::isnan(residual_num) || fabs(residual_num) > MAX_RESIDUAL) 
        {
            LOG(Level::LERROR, "Residual out of bounds or NaN detected, aborting.");
            throw std::runtime_error("Residual out of bounds or NaN detected");
        }
        /* Check if the termination criteria based on the residual change is met */
        if (fabs(residual_num - prev_residual) < TERMINATION_CRITERIA)        
        {
            LOG(Level::LINFO, "Termination criteria reached.");
            /* Store the closest point on the curve */
            closest_approach.x = x;
            closest_approach.y = eigen::Compute::evaluate(x, coeff.a0, coeff.a1, coeff.a2, coeff.a3);
            /* Fill the Frenet frame with the closest point and lateral distance */
            frame.closest_point_on_curve = closest_approach;
            frame.lateral_distance = residual_num;
            return frame; 
        }
        else
        {
            /* Update the previous residual for the next iteration */
            prev_residual = residual_num;
        }
    }
    /* If the maximum number of iterations is reached without convergence, return the last known point */
    LOG(Level::LERROR, "Maximum number of iterations exceeded.");
    closest_approach.x = x;
    closest_approach.y = eigen::Compute::evaluate(x, coeff.a0, coeff.a1, coeff.a2, coeff.a3);
    /* Fill the Frenet frame with the closest point and lateral distance */
    frame.closest_point_on_curve = closest_approach;
    frame.lateral_distance = residual_num;
    return frame;
}

/**
 * Begin spline spline spline spline spline spline spline spline spline spline spline spline spline spline spline 
 */

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

/* Eof */