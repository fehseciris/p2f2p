#include "num.h"

double NumMethods::eucledian_distance(const sPoint A, const sPoint B)
{
    double diffY = B.y - A.y;
    double diffX = B.x - A.x;
    return sqrt(diffY * diffY + diffX * diffX);
}

double NumMethods::cross_product(const sPoint A, const sPoint B)
{
    return ((A.x * B.y) - (A.y * B.x));
}

double NumMethods::evaluate(const double x, const double a1, const double a2, const double a3)
{
    return ((a1 * x * x) + (a2 * x) + a3);
}

double NumMethods::gradient(const double x, const double p, const double q, const double a1, const double a2, const double a3)
{
    double a1_cubed = a1 * a1 * a1;
    double a2_squared = a2 * a2;
    double x_squared = x * x;
    double x_cubed = x * x * x;
    return (-(4 * a1_cubed * x_cubed) - (6 * a1 * a2 * x_squared) - (2 * x * (a2_squared - 1 + (2 * a1 * (a3 - q)))) + (-2 * p - 2 * a2 * a3 - 2 * q * a2));
}

double NumMethods::residual(const double x, const double p, const double q, const double a1, const double a2, const double a3)
{
    double diffQ = q - evaluate(x, a1, a2, a3);
    double diffP = p - x;
    return (diffQ * diffQ + diffP * diffP);
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
    /* Normalize the x-values to the range [0, 1] */
    double x_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::min();
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::min();

    /* Find the minimum and maximum x- and y-values */
    for (const auto& point : points) {
        if (point.x < x_min) x_min = point.x;
        if (point.x > x_max) x_max = point.x;
        if (point.y < y_min) y_min = point.y;
        if (point.y > y_max) y_max = point.y;
    }

    double x_range = x_max - x_min;
    double y_range = y_max - y_min;

    /* Avoid division by zero */
    if (x_range == 0) x_range = 1.0;
    if (y_range == 0) y_range = 1.0;

    Eigen::MatrixXd A(points.size(), order + 1);
    Eigen::VectorXd yv(points.size());
    Eigen::VectorXd result;

    /* Fill the matrix A and the vector yv */
    for (int i = 0; i < points.size(); i++) 
    {
        /* Normalize the x and y values */
        double x_normalized = (points[i].x - x_min) / x_range;
        double y_normalized = (points[i].y - y_min) / y_range;
        yv(i) = y_normalized;

        /* Use direct multiplication for x^j */
        double x_power = 1.0;  /* Start with x^0 */
        for (int j = 0; j <= order; j++) {
            A(i, j) = x_power;             /* Set x^j */
            x_power *= x_normalized;       /* Increment the power */
        }
    }

    /* Use Singular Value Decomposition (SVD) for a more stable solution */
    result = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(yv);

    /* Denormalize the coefficients */
    coeff.resize(order + 1);
    for (int i = 0; i <= order; i++) 
    {
        /* Denormalize coefficients based on the scaling of x and y values */
        coeff[i] = result[i] * (y_range / std::pow(x_range, i));
    }
    
    /* If necessary, adjust the constant term (a0) separately */
    coeff[0] = coeff[0] * y_range + y_min;
    return;
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
    double x = curve.points[init_index].x;  
    double y = curve.points[init_index].y;
    double p = target.x;  
    double q = target.y;
    double prev_residual = VERY_LARGE_NUMBER;
    double residual_ = 0;
    
    double min_alpha = 1e-10;  /* Minimum value for the learning rate to prevent it from getting too small */
    double max_grad = 1e10;    /* Maximum allowed gradient value */
    double max_residual = 1e200; /* Maximum allowed residual value */

    sPoint closest_approach;
    sFrenet frame;
    frame.cartesian_point = target;
    
    for(int i = 0; i < max_iter; i++)
    {
        std::cout << "x: " << x << ", a1: " << curve.coefficients[2] 
          << ", a2: " << curve.coefficients[1] 
          << ", a3: " << curve.coefficients[0] 
          << ", p: " << p << ", q: " << q << std::endl;
          
        /* Calculate the gradient */
        double grad = gradient(x, p, q, curve.coefficients[2], curve.coefficients[1], curve.coefficients[0]);
        std::cout << "grad: " << grad;
        
        /* If the gradient is NaN or exceeds the maximum allowed value, abort */
        if (std::isnan(grad) || fabs(grad) > max_grad) 
        {
            LOG(Level::LERROR, "Gradient out of bounds or NaN detected, aborting.");
            throw std::runtime_error("Gradient out of bounds or NaN detected");
        }
        
        /* Dynamically adjust the learning rate if the gradient is too large */
        if (fabs(grad) > 1e5)
        {
            alpha /= 2;  /* Reduce the learning rate when the gradient is too large */
            if (alpha < min_alpha) 
            {
                LOG(Level::LERROR, "Learning rate too small, aborting.");
                throw std::runtime_error("Learning rate too small");
            }
        }

        /* Update x using the gradient */
        x = x - (alpha * grad);

        /* Calculate the new residual */
        residual_ = residual(x, p, q, curve.coefficients[2], curve.coefficients[1], curve.coefficients[0]);
        std::cout << " | residual: " << residual_ << std::endl;

        /* If the residual is NaN or exceeds the maximum allowed value, abort */
        if (std::isnan(residual_) || fabs(residual_) > max_residual) 
        {
            LOG(Level::LERROR, "Residual out of bounds or NaN detected, aborting.");
            throw std::runtime_error("Residual out of bounds or NaN detected");
        }

        /* Check if the termination criteria based on the residual change is met */
        if (fabs(residual_ - prev_residual) < TERMINATION_CRITERIA)        
        {
            LOG(Level::LINFO, "Termination criteria reached.");
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

    /* If the maximum number of iterations is exceeded */
    LOG(Level::LERROR, "Maximum number of iterations exceeded.");
    closest_approach.x = x;
    closest_approach.y = y;
    frame.closest_point_on_curve = closest_approach;
    frame.lateral_distance = residual_;
    return frame;
}

/* Eof */