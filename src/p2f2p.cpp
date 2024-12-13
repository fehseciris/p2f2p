#include "p2f2p.h"

/**
 * Default constructor for P2F2P. Initializes the object without any waypoints and logs the initialization.
 */
spline::P2F2P::P2F2P()
{
    LOG(Level::LINFO, Topic::P2F2P, "Init object without waypoints");
}

/**
 * Constructor for P2F2P. Initializes the object with a vector of waypoints.
 * Throws an exception if the number of waypoints exceeds the maximum or minimum limit.
 * @param input A vector of waypoints (sPoint objects) to initialize the object.
 * @throws std::out_of_range If the number of waypoints exceeds max or is below the min limit.
 */
spline::P2F2P::P2F2P(const std::vector<sPoint>& input)
{
    if(input.size() > MAX_NUM_WAYPOINTS && input.size() < MIN_NUM_WAYPOINTS)
    {
        throw std::out_of_range("Reach max or min waypoints limit");
    }
    this->init();
    this->points_ = input;
    LOG(Level::LINFO, Topic::P2F2P, "Init object with waypoints - size: " + std::to_string(points_.size()));
    this->pre_calculator();
}

/**
 * Destructor for P2F2P. Cleans up internal data by resetting to initial state.
 */
spline::P2F2P::~P2F2P()
{
    /* Clean up */
    this->init();
    LOG(Level::LDEBUG, Topic::P2F2P, "Destroy object");
}

/**
 * Updates the object's waypoints and processes them.
 * Clears existing data, validates the input size, and performs pre-calculations.
 * @param input A vector of sPoint objects representing the new waypoints.
 * @throws std::out_of_range If the number of waypoints exceeds max or is below the min limit.
 */
void spline::P2F2P::process_points(const std::vector<sPoint>& input)
{
    if(input.size() > MAX_NUM_WAYPOINTS && input.size() < MIN_NUM_WAYPOINTS)
    {
        throw std::out_of_range("Reach max or min waypoints limit.");
    }
    this->init();
    this->points_ = input;
    LOG(Level::LINFO, Topic::P2F2P, "Refresh object with new waypoints - size: " + std::to_string(points_.size()));
    this->pre_calculator();
    return;
}

/**
 * Converts a Cartesian point to its Frenet frame representation.
 * Calculates and sets the closest point on the curve, geodetic distance, and lateral distance.
 * @note The spline plot is only available starting the program from p2f2p/build/debug
 * you need to check this before. Maybe you need to change the path.
 * @param point The Cartesian coordinates of the input point.
 * @return The Frenet frame of the target point as an sFrenet object.
 */
sFrenet spline::P2F2P::g2f(const sPoint& point)
{
    this->target_ = point;
    if(PYTHON_PLOTS)
    {
        points_to_file("../../plot/waypoints.txt", this->points_, this->target_);
        LOG(Level::LDEBUG, Topic::P2F2P, "Start spline plot");
        system("start cmd /k python ../../plot/spline.py");
    }
    this->frenet_.cartesian_point = point;
    this->frenet_.closest_cartesian_point = this->closest_waypoint(point);
    this->frenet_.closest_point_on_curve = this->closest_point_on_curve(point);
    this->frenet_.geodetic_distance = this->geodetic_distance(this->frenet_.closest_point_on_curve);
    this->frenet_.lateral_distance = spline::Compute::eucledian_distance(this->frenet_.closest_point_on_curve, point);
    this->frenet_.direction = direction(point);
    LOG(Level::LINFO, Topic::P2F2P, "Kartesian to frenet transformation complete");
    return this->frenet_;
}

/**
 * Converts a Frenet frame point back to Cartesian coordinates.
 * Calculates the corresponding point on the curve.
 * @param frenet The Frenet frame (sFrenet object) to convert.
 * @return The Cartesian coordinates of the Frenet frame as an sPoint object.
 */
sPoint spline::P2F2P::f2g(const sFrenet& frenet)
{
    this->frenet_ = frenet;
    /* Find closest point on curve. */
    sPoint target = {0,0};
    sPoint actual = {0,0};
    sPoint last = {this->sx_(this->extract_T_.front()), this->sy_(this->extract_T_.front())};
    double distance_between = 0.0;
    double actual_distance = this->frenet_.geodetic_distance;
    double dummy = 0;
    for(double t = this->extract_T_.front() + DISCRETIZATION_DISTANCE; 
            t <= this->extract_T_.back(); t += DISCRETIZATION_DISTANCE)
    {
        dummy++;
        actual = {this->sx_(t) + DISCRETIZATION_DISTANCE, this->sy_(t) + DISCRETIZATION_DISTANCE};
        distance_between = spline::Compute::eucledian_distance(last, actual);
        actual_distance -= distance_between;
        if(actual_distance < 0)
        {
            /* Found closest point on curve */
            target.x = this->sx_(t);
            target.y = this->sy_(t);
            break;
        }
        last = actual;
    }
    /* Find point in cartesian with lateral distance */
    /* Vector for direction */
    double dir_x = actual.x - last.x;
    double dir_y = actual.y - last.y;
    /* Normal vector */
    double normal_x;
    double normal_y;
    if (this->frenet_.direction) 
    {   
        /* 90 deg left */
        normal_x = -dir_y;
        normal_y = dir_x;
    } 
    else 
    {   
        /* 90 def right */
        normal_x = dir_y;
        normal_y = -dir_x;
    }
    /* Normalize normal vector */
    double length = std::sqrt(normal_x * normal_x + normal_y * normal_y);
    normal_x /= length;
    normal_y /= length;
    /* Calculate point in lateral distance */
    target.x = actual.x + this->frenet_.lateral_distance * normal_x;
    target.y = actual.y + this->frenet_.lateral_distance * normal_y;
    LOG(Level::LINFO, Topic::P2F2P, "Frenet to kartesian transformation complete");
    return target;
}

/**
 * Calculates the total length of the path by summing the distances between discrete points on the curve.
 * @return The total path length as a double.
 */
double spline::P2F2P::path_length(void)
{
    /* Start by getting the first point on the curve */
    sPoint actual = {0,0};
    sPoint last = {this->sx_(this->extract_T_.front()),
                    this->sy_(this->extract_T_.front())};
    double path_length = 0;
    /* Iterate over the curve by stepping through the parameter 't' */
    for(double t = this->extract_T_.front() + DISCRETIZATION_DISTANCE; 
            t <= this->extract_T_.back(); t += DISCRETIZATION_DISTANCE)
    {
        /* Compute the coordinates of the point on the curve at parameter 't' */
        actual = {this->sx_(t), this->sy_(t)};
        /* Compute the distance between the current point on the curve and the last point */
        path_length += spline::Compute::eucledian_distance(actual, last);
        last = actual;
    }
    LOG(Level::LDEBUG, Topic::P2F2P, "Path length: " + std::to_string(path_length));
    LOG(Level::LINFO, Topic::P2F2P, "Path length calculation complete");
    return path_length;
}

/**
 * Finds the Cartesian position on the curve at a specified distance along the curve.
 * @param distance The distance along the curve for which the position is calculated.
 * @return The position on the curve as an sPoint object.
 */
sPoint spline::P2F2P::position(double& distance)
{
    sPoint target = {0,0};
    sPoint actual = {0,0};
    sPoint last = {this->sx_(this->extract_T_.front()), this->sy_(this->extract_T_.front())};
    double distance_between = 0.0;
    double actual_distance = distance;
    double dummy = 0;
    for(double t = this->extract_T_.front() + DISCRETIZATION_DISTANCE; 
            t <= this->extract_T_.back(); t += DISCRETIZATION_DISTANCE)
    {
        dummy++;
        actual = {this->sx_(t) + DISCRETIZATION_DISTANCE, this->sy_(t) + DISCRETIZATION_DISTANCE};
        distance_between = spline::Compute::eucledian_distance(last, actual);
        actual_distance -= distance_between;
        if(actual_distance < 0)
        {
            target.x = this->sx_(t);
            target.y = this->sy_(t);
            break;
        }
        last = actual;
    }
    LOG(Level::LDEBUG, Topic::P2F2P, "Point after " + std::to_string(distance)
            + " - " + pto_string(target));
    LOG(Level::LINFO, Topic::P2F2P, "Position calculation complete");
    return target;
}   

/**
 * Calculates the tangent angle at a specified distance along the curve, in degrees.
 * @param distance The distance along the curve for which the tangent angle is calculated.
 * @return The tangent angle in degrees as a double.
 */
double spline::P2F2P::tangent_angle_deg(double& distance)
{
    /* Use angle in rad and convert to degree */
    double angle = this->_tangent_angle_rad(distance) * (180 / M_PI);
    LOG(Level::LDEBUG, Topic::P2F2P, "Tangent angle after " + std::to_string(distance) 
            + " - " + std::to_string(angle) + " in deg");
    LOG(Level::LINFO, Topic::P2F2P, "Tangent angle calculation in deg complete");
    return angle;
}

/**
 * Calculates the tangent angle at a specified distance along the curve, in radians.
 * @param distance The distance along the curve for which the tangent angle is calculated.
 * @return The tangent angle in radians as a double.
 */
double spline::P2F2P::tangent_angle_rad(double& distance)
{
    double dummy = this->_tangent_angle_rad(distance);
    LOG(Level::LINFO, Topic::P2F2P, "Tangent angle calculation in rad complete");
    return dummy;
}

/**
 * Calculates the curvature at a specified distance along the curve.
 * @param distance The distance along the curve for which the curvature is calculated.
 * @return The curvature as a double.
 */
double spline::P2F2P::curvature(double& distance)
{
    double dummy = this->_curvature(distance);
    LOG(Level::LINFO, Topic::P2F2P, "Curvature calculation complete");
    return dummy;
}

/**
 * Calculates the change in curvature with respect to the arc length at a specified distance.
 * @param distance The distance along the curve for which the change in curvature is calculated.
 * @return The rate of change in curvature as a double.
 */
double spline::P2F2P::change_in_curvature(double& distance)
{
    double distance_plus = distance + DISCRETIZATION_DISTANCE;
    double distance_minus = distance - DISCRETIZATION_DISTANCE;
    double kappa_1 = this->_curvature(distance_minus);
    double kappa_2 = this->_curvature(distance_plus);
    double dkappa_ds = (kappa_2 - kappa_1) / (2 * DISCRETIZATION_DISTANCE);
    LOG(Level::LDEBUG, Topic::P2F2P, "Change in curvature after " + std::to_string(distance) +
            " - " + std::to_string(dkappa_ds));
    LOG(Level::LINFO, Topic::P2F2P, "Change in curvature calculation complete");
    return dkappa_ds;
}

/**
 * Calculates the tangent angle at a specified distance along the curve, in radians.
 * @param distance The distance along the curve for which the tangent angle is calculated.
 * @return The tangent angle in radians as a double.
 */
double spline::P2F2P::_tangent_angle_rad(double& distance)
{
    /* Calculate derivative at distance */
    sPoint derv_point = {this->sx_.deriv(1, distance), this->sy_.deriv(1, distance)};
    /* Calculate angle in rad */
    double angle = std::atan2(derv_point.y, derv_point.x);
    LOG(Level::LDEBUG, Topic::P2F2P, "Tangent angle after " + std::to_string(distance)
            + " - " + std::to_string(angle) + " in rad");
    return angle;
}

/**
 * Calculates the curvature at a specified distance along the curve.
 * @param distance The distance along the curve for which the curvature is calculated.
 * @return The curvature as a double.
 */
double spline::P2F2P::_curvature(double& distance)
{
    double dx = this->sx_.deriv(1, distance);
    double dy = this->sy_.deriv(1, distance);
    double ddx = this->sx_.deriv(2, distance);
    double ddy = this->sy_.deriv(2, distance);
    /* Calculate numerator */
    double numerator = dx * ddy - dy * ddx; 
    /* Calculate denominator */
    double denominator = std::pow(dx * dx + dy * dy, 1.5);
    /* Divide with null */
    if (denominator == 0) 
    {
        throw std::runtime_error("Divide by null in curvature calculation.");
    }
    LOG(Level::LDEBUG, Topic::P2F2P, "Curvature after " + std::to_string(distance) 
            + " - " + std::to_string(numerator / denominator));
    return numerator / denominator;
}

/**
 * Prepares the curve for calculations by interpolating x(t) and y(t) values.
 * Sets up internal parameters based on input waypoints, computes spline interpolation, and saves data for plotting.
 */
void spline::P2F2P::pre_calculator(void)
{   
    LOG(Level::LDEBUG, Topic::P2F2P, "Enter pre calculator");
    /* Parameter calculator */
    double t = 0; 
    double d = 0;
    /* Ensure the first point is handled correctly */
    if (this->points_.size() > 0)
    {
        this->extract_D_.push_back(0); // Distance to the first point is 0
        this->extract_T_.push_back(0); // First point, t = 0
        this->extract_X_.push_back(this->points_[0].x); // X of the first point
        this->extract_Y_.push_back(this->points_[0].y); // Y of the first point
    }
    /* Start the loop from the second point (i = 1) */
    for(size_t i = 1; i < this->points_.size(); i++)
    {
        d = spline::Compute::eucledian_distance(this->points_[i], this->points_[i - 1]);
        t += d;
        this->extract_D_.push_back(d);  
        this->extract_T_.push_back(t);
        this->extract_X_.push_back(this->points_[i].x);
        this->extract_Y_.push_back(this->points_[i].y);
    }
    LOG(Level::LDEBUG, Topic::P2F2P, "Input waypoints size: " + std::to_string(points_.size()));
    LOG(Level::LDEBUG, Topic::P2F2P, "Extracted T's size: " + std::to_string(extract_T_.size())); 
    LOG(Level::LDEBUG, Topic::P2F2P, "Extracted X's size: " + std::to_string(extract_X_.size())); 
    LOG(Level::LDEBUG, Topic::P2F2P, "Extracted Y's size: " + std::to_string(extract_Y_.size())); 
    /* Spline interpolation for x(t) and y(t) */
    this->sx_.set_points(this->extract_T_, this->extract_X_);
    this->sy_.set_points(this->extract_T_, this->extract_Y_);
    LOG(Level::LINFO, Topic::P2F2P, "Spline interpolation complete.");
    if(PYTHON_PLOTS)
    {
        std::vector<sPoint> points;
        for(double t = this->extract_T_.front(); t <= this->extract_T_.back(); t += DISCRETIZATION_DISTANCE)
        {
            /* Compute the coordinates of the point on the curve at parameter 't' */
            points.push_back({this->sx_(t), this->sy_(t)});
        }
        points_to_file("../../plot/spline.txt", points);
    }
    return;
}

/**
 * Finds the closest waypoint to a given target point.
 * @param target The Cartesian coordinates of the target point.
 * @return The sPoint object representing the closest waypoint.
 */
sPoint spline::P2F2P::closest_waypoint(const sPoint& target)
{
    sPoint closest;
    double min_distance = std::numeric_limits<double>::max(); 
    for(size_t i = 1; i < this->points_.size(); i++)
    {
        double distance = spline::Compute::eucledian_distance(this->points_[i], target);
        if(distance < min_distance)  
        {
            min_distance = distance; 
            closest = this->points_[i];
        }
    }
    LOG(Level::LDEBUG, Topic::P2F2P, "Closest waypoint - " + pto_string(closest));
    return closest;
}

/**
 * Finds the closest point on the continuous curve to a given target point.
 * @param target The Cartesian coordinates of the target point.
 * @return The sPoint object representing the closest point on the curve.
 */
sPoint spline::P2F2P::closest_point_on_curve(const sPoint& target)
{
    sPoint closest; 
    sPoint actual;
    double min_distance = std::numeric_limits<double>::max(); 
    /* Start by getting the first point on the curve */
    double prev_x = this->sx_(this->extract_T_.front());
    double prev_y = this->sy_(this->extract_T_.front());
    /* Iterate over the curve by stepping through the parameter 't' */
    for(double t = this->extract_T_.front(); t <= this->extract_T_.back(); t += DISCRETIZATION_DISTANCE)
    {
        /* Compute the coordinates of the point on the curve at parameter 't' */
        actual = {this->sx_(t), this->sy_(t)};
        /* Compute the distance between the current point on the curve and the target point */
        double distance = spline::Compute::eucledian_distance(actual, target);
        /* If this distance is smaller than the previously recorded minimum distance, update the closest point */
        if(distance < min_distance)
        {
            min_distance = distance;
            closest.x = actual.x;
            closest.y = actual.y;
        }
        /* Update the previous coordinates for the next iteration */
        prev_x = actual.x;
        prev_y = actual.y;
    }
    LOG(Level::LDEBUG, Topic::P2F2P, "Closest point on spline - " + pto_string(closest));
    return closest;
}

/**
 * Calculates the geodetic distance from the origin along the curve to the closest point on the curve to the target.
 * @param target The Cartesian coordinates of the closest point on the curve.
 * @return The geodetic distance to the target as a double.
 */
double spline::P2F2P::geodetic_distance(const sPoint& target)
{
    double total_distance = 0;
    double min_distance_to_target = std::numeric_limits<double>::max();
    double geodetic_distance_to_target = 0;
    sPoint prev = {this->sx_(this->extract_T_.front()), this->sy_(this->extract_T_.front())};
    for(double t = this->extract_T_.front() + DISCRETIZATION_DISTANCE; 
            t <= this->extract_T_.back(); t += DISCRETIZATION_DISTANCE)
    {
        sPoint actual = {this->sx_(t), this->sy_(t)};
        /* Calculate distance between actual point and target point */
        double distance_to_target = spline::Compute::eucledian_distance(actual, target);
        /* Actualize the geodetic distance if new point is closer */
        if (distance_to_target < min_distance_to_target)
        {
            min_distance_to_target = distance_to_target;
            geodetic_distance_to_target = total_distance;  
        }
        /* Add geodetic distances */
        total_distance += spline::Compute::eucledian_distance(prev, actual);
        prev.x = actual.x;
        prev.y = actual.y;
    }
    LOG(Level::LDEBUG, Topic::P2F2P, "Geodetic distance - " + std::to_string(geodetic_distance_to_target));
    return geodetic_distance_to_target;
}

/**
 * Determines whether the target point is to the left or right of the curve based on the Frenet frame.
 * @param target The Cartesian coordinates of the target point.
 * @return True if the point is to the right, false if to the left.
 * @throws std::runtime_error If the target point is on the curve.
 */
bool spline::P2F2P::direction(const sPoint& target)
{
    sPoint closest_on_curve = this->frenet_.closest_point_on_curve;
    sPoint actual = {0,0};
    sPoint last = {0,0};
    for(double t = this->extract_T_.front() + DISCRETIZATION_DISTANCE; 
            t <= this->extract_T_.back(); t += DISCRETIZATION_DISTANCE)
    {
        actual = {this->sx_(t), this->sy_(t)};
        if(closest_on_curve == actual)
        {
            break;
        }
        last = actual;
    }
    double cross_product = spline::Compute::cross_product(closest_on_curve, last, target);
    if (cross_product > 0) 
    {
        /* Right */
        LOG(Level::LDEBUG, Topic::P2F2P, "Target direction from spline - right");
        return true;  
    } 
    else if (cross_product < 0)
    {
        /* Left */
        LOG(Level::LDEBUG, Topic::P2F2P, "Target direction from spline - left");
        return false; 
    }
    else
    {
        throw std::runtime_error("Target on spline.");
    }
}

/**
 * Resets the internal state of the object, clearing stored waypoints, Frenet data, and interpolation parameters.
 */
void spline::P2F2P::init(void)
{
    this->points_.clear();
    this->frenet_.cartesian_point = {0.0,0.0};
    this->frenet_.closest_point_on_curve = {0.0,0.0};
    this->frenet_.closest_cartesian_point = { 0.0,0.0 };
    this->frenet_.geodetic_distance = 0.0;
    this->frenet_.lateral_distance = 0.0;
    this->target_ = {0.0,0.0};
    this->extract_X_.clear();
    this->extract_Y_.clear();
    this->extract_T_.clear();
    return;
}

/* Extern */

/**
 * Overloaded operator<< to output the state of a P2F2P object to an output stream.
 * Prints settings, waypoints, and Frenet information.
 * @param os The output stream to which the P2F2P object will be written.
 * @param o The P2F2P object to output.
 * @return The modified output stream.
 */
std::ostream& spline::operator<<(std::ostream& os, const P2F2P& o)
{
    os  << "***** Spline object ****************************************************\n"
        << "*\n"  
        << "* SETTINGS\n"    
        << "* -> Discretization distance:               " << DISCRETIZATION_DISTANCE << "\n"
        << "* -> Min waypoints                          " << MIN_NUM_WAYPOINTS << "\n"
        << "* -> Max waypoints                          " << MAX_NUM_WAYPOINTS << "\n"
        << "* -> Number of waypoints                    " << o.points_.size() << "\n"
        << "*\n"
        << "* INTERN\n"
        << "* -> point_:                                " << o.point_ << "\n"
        << "* -> target_:                               " << o.target_ << "\n"
        << "* -> frenet_.cartesian_point                " << o.frenet_.cartesian_point << "\n"
        << "* -> frenet_.closest_cartestian_point:      " << o.frenet_.closest_cartesian_point << "\n"
        << "* -> frenet_.closest_point_on_curve:        " << o.frenet_.closest_point_on_curve << "\n"
        << "* -> frenet_.geodetic_distance              " << o.frenet_.geodetic_distance << "\n"
        << "* -> frenet_.direction                      " << o.frenet_.direction << "\n"
        << "* -> frenet_.lateral_distance               " << o.frenet_.lateral_distance << "\n"
        << "*\n"
        << "***** End **************************************************************\n";
    return os;
}


/* Eof */