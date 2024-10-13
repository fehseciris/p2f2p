#include "num.h"

/**
 * Begin eigen eigen eigen eigen eigen eigen eigen eigen eigen eigen eigen eigen eigen eigen eigen 
 */

double eigen::Compute::eucledian_distance(const sPoint A, const sPoint B)
{
    double diffY = B.y - A.y;
    double diffX = B.x - A.x;
    return sqrt(diffY * diffY + diffX * diffX);
}

double eigen::Compute::cross_product(const sPoint A, const sPoint B)
{
    return ((A.x * B.y) - (A.y * B.x));
}

double eigen::Compute::evaluate(const double x, const double a0, const double a1, const double a2, const double a3 = 0)
{
    return ((a3 * x * x * x) + (a2 * x * x) + (a1 * x) + a0);
}

double eigen::Compute::gradient(const double x, const double p, const double q, const double a0, const double a1, const double a2, const double a3 = 0)
{
    double grad_f_x = 3 * a3 * x * x + 2 * a2 * x + a1;
    double df_dx = 2 * (x - p) + 2 * (eigen::Compute::evaluate(x, a0, a1, a2, a3) - q) * grad_f_x;
    return df_dx;
}

double eigen::Compute::residual(const double x, const double p, const double q, const double a0, const double a1, const double a2, const double a3 = 0)
{
    double diffQ = q - eigen::Compute::evaluate(x, a0, a1, a2, a3); 
    double diffP = p - x;
    return sqrt(diffQ * diffQ + diffP * diffP);
}

/**
 * Fitting a polynomial to the input points (uses the Eigen library)
 *
 * @param points Input waypoints
 * @param coeff Output coefficients of the polynomial equation
 * @param order Required order of the polynomial
 * @return void
 */
void eigen::Compute::poly_fit_points(const std::vector<sPoint>& points, std::vector<sCoeff>& coeffs, int order)
{
    std::ofstream file("coefficients.txt");
    if (!file.is_open()) 
    {
        std::cerr << "Unable to open file to write coefficients" << std::endl;
        return;
    }

    for (int i = order - 1; i < points.size() - (size_t)(order - 1); ++i) 
    {
        // Select 5 points (2 previous, 1 current, 2 next)
        std::vector<sPoint> sub_points(points.begin() + i - (order - 1), points.begin() + i + (order - 1));

        // Find the minimum and maximum x-values (without normalizing)
        double x_min = std::numeric_limits<double>::min();
        double x_max = std::numeric_limits<double>::max();

        for (const auto& point : sub_points) 
        {
            if (point.x < x_min) x_min = point.x;
            if (point.x > x_max) x_max = point.x;
        }

        double x_range = x_max - x_min;

        // Avoid division by zero
        if (x_range == 0) x_range = 1.0;

        // Define matrices for polynomial fitting
        Eigen::MatrixXd A(sub_points.size(), order + 1);
        Eigen::VectorXd yv(sub_points.size());
        Eigen::VectorXd result;

        // Fill the matrix A and the vector yv
        for (int j = 0; j < sub_points.size(); j++) 
        {
            double x = sub_points[j].x;  // Use original x-values
            double y = sub_points[j].y;

            yv(j) = y;  // Keep original y-values without normalization

            // Use direct multiplication for x^j
            double x_power = 1.0; // Start with x^0
            for (int k = 0; k <= order; k++) 
            {
                A(j, k) = x_power; // Set x^k
                x_power *= x;      // Increment the power
            }
        }

        // Use Singular Value Decomposition (SVD) for a more stable solution
        result = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(yv);

        // Store the coefficients in sCoeff structure
        sCoeff coeff;
        for (int k = 0; k <= order; k++) 
        {
            if (k == 0) 
                coeff.a0 = result[k];  // Constant term
            if (k == 1) 
                coeff.a1 = result[k];  // Linear term
            if (k == 2) 
                coeff.a2 = result[k];  // Quadratic term
            if (k == 3) 
                coeff.a3 = result[k];  // Cubic term (if order = 3)
        }

        coeffs.push_back(coeff);

        // Write the coefficients to the file
        file << coeff.a0 << "," << coeff.a1 << "," << coeff.a2 << "," << coeff.a3 << std::endl;
    }

    file.close();
    // Python-Skript ausführen
    system("start python coefficients.py"); 
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
sFrenet eigen::Compute::steepest_gradient_descent(const sCurve& curve, const sPoint& target, int init_index, double alpha, int max_iter)
{
    // Initial values
    double x = curve.points[init_index].x;  // Start with the x-coordinate of the nearest point
    double y = curve.points[init_index].y;  // Corresponding y-coordinate
    double p = target.x;  // Target x-coordinate
    double q = target.y;  // Target y-coordinate
    double prev_residual = std::numeric_limits<double>::max();;  // Start with a large residual value
    double residual_num = 0;

    sPoint closest_approach;  // To store the closest point on the curve
    sFrenet frame;  // To store the final result in Frenet coordinates
    frame.cartesian_point = target;  // Set the target point in the Frenet frame

    // Ensure the index is valid (if init_index is too small, default to 2)
    if (init_index < 2) 
    {
        init_index = 2;
    }

    // Retrieve the polynomial coefficients for the current segment
    const sCoeff& coeff = curve.coefficients[init_index];  // Extract the polynomial coefficients for this segment
    std::cout << "used " << coeff;

    // Iterative optimization (gradient descent)
    for (int i = 0; i < max_iter; i++) 
    {
        std::cout << "x: " << x
                << " | p: " << p 
                << " | q: " << q
                << std::endl;

        // Calculate the gradient using the polynomial's coefficients (a1, a2, a3)
        double grad = eigen::Compute::gradient(x, p, q, coeff.a0, coeff.a1, coeff.a2, coeff.a3);
        std::cout << "grad: " << grad;

        // If the gradient is NaN or exceeds a defined threshold, abort
        if (std::isnan(grad) || fabs(grad) > MAX_GRADIENT) 
        {
            LOG(Level::LERROR, "Gradient out of bounds or NaN detected, aborting.");
            throw std::runtime_error("Gradient out of bounds or NaN detected");
        }

        // Update the x value using the gradient with a constant learning rate (alpha)
        x = x - (alpha * grad);

        // Calculate the new residual (distance between the new point and the target point)
        residual_num = eigen::Compute::residual(x, p, q, coeff.a0, coeff.a1, coeff.a2, coeff.a3);
        std::cout << " | residual: " << residual_num << std::endl;

        // If the residual is NaN or exceeds a defined threshold, abort
        if (std::isnan(residual_num) || fabs(residual_num) > MAX_RESIDUAL) 
        {
            LOG(Level::LERROR, "Residual out of bounds or NaN detected, aborting.");
            throw std::runtime_error("Residual out of bounds or NaN detected");
        }

        // Check if the termination criteria based on the residual change is met
        if (fabs(residual_num - prev_residual) < TERMINATION_CRITERIA)        
        {
            LOG(Level::LINFO, "Termination criteria reached.");

            // Store the closest point on the curve
            closest_approach.x = x;
            closest_approach.y = eigen::Compute::evaluate(x, coeff.a0, coeff.a1, coeff.a2, coeff.a3);  // Calculate y using the polynomial

            // Fill the Frenet frame with the closest point and lateral distance
            frame.closest_point_on_curve = closest_approach;
            frame.lateral_distance = residual_num;

            return frame;  // Return the Frenet frame with the result
        }
        else
        {
            // Update the previous residual for the next iteration
            prev_residual = residual_num;
        }
    }

    // If the maximum number of iterations is reached without convergence, return the last known point
    LOG(Level::LERROR, "Maximum number of iterations exceeded.");
    closest_approach.x = x;
    closest_approach.y = eigen::Compute::evaluate(x, coeff.a0, coeff.a1, coeff.a2, coeff.a3);  // Calculate y using the polynomial

    // Fill the Frenet frame with the closest point and lateral distance
    frame.closest_point_on_curve = closest_approach;
    frame.lateral_distance = residual_num;

    return frame;
}

/**
 * Begin spline spline spline spline spline spline spline spline spline spline spline spline spline spline spline 
 */



/* Eof */