#include "p2f2p.h"

std::vector<sPoint> collect(int argc, char* argv[])
{
    if(argc > 1)
    {
        if (std::string(argv[1]) == "-p") 
        {
            if((argc - 1) % 2 != 0)
            {
                throw std::invalid_argument("Invalid amount of arguments.");
            }
            std::vector<sPoint> vector;
            sPoint dummy;
            for (int i = 2; i < argc; i += 2) 
            {
                try 
                {
                    dummy.x = std::stoi(argv[i]);
                    dummy.y = std::stoi(argv[i + 1]);
                } 
                catch (const std::invalid_argument& e) 
                {
                    throw std::invalid_argument("Invalid argument: non-numeric value.");
                } 
                catch (const std::out_of_range& e) 
                {
                    throw std::out_of_range("Argument out of range.");
                }
                vector.push_back(dummy);
            }
            return vector;
        } 
        else
        {
            throw std::invalid_argument("Invalid arguments.");
        }
    }
    else
    {
        throw std::runtime_error("Arguments empty.");
    }
}

/**
 * begin eigen eigen eigen eigen eigen eigen eigen eigen eigen eigen eigen eigen eigen eigen eigen
 */

eigen::P2F2P::P2F2P()
{
    LOG(Level::LINFO, "Init object without waypoints");
}

eigen::P2F2P::P2F2P(const std::vector<sPoint>& points)
{
    LOG(Level::LINFO, "Init object with waypoints");
    if(points.size() < 10)
    {
        throw std::runtime_error("Not enough points! Min 10.");
    }
    this->waypoints_ = points;
    /* obtain the equation of the curve */
    this->curve_ = obtain_curve();
}

eigen::P2F2P::~P2F2P()
{
    this->clear();
}

void eigen::P2F2P::process_points(const std::vector<sPoint>& points)
{
    LOG(Level::LINFO, "Refresh object with point data.");
    if(points.size() < 10)
    {
        throw std::runtime_error("Not enough points! Min 10.");
    }
    this->clear();
    this->waypoints_ = points;
    /* obtain the equation of the curve */
    this->curve_ = obtain_curve();
    return;
}

/**
 * Processes the input points and generates the Frenet coodinates.
 * This is the master-fuction triggering all the sub-modules required for solving the problem
 *
 * @param target Cartesian coordinate of input point
 * @return void
 */
sFrenet eigen::P2F2P::g2f(const sPoint& target)
{
    this->point_ = target;
    /* Find the point among the input waypoints that lies closest to the input/target point */
    int closest_point = find_nearest_point(target);            
    /* Find the closest point to the input/target point on the continuous curve and get the distance */
    sFrenet frame = distance_to_curve(target, closest_point);   
    /* Find the distance to the closest point on the continuous curve from the curve's origin */
    geodetic_distance(frame, closest_point);    
    this->frenet_ = frame;               
    return frame;                        
}

sPoint eigen::P2F2P::f2g(const sFrenet& target)
{
    sPoint point;
    return point;
}

double eigen::P2F2P::path_length(void)
{
    return 0.;
}

sPoint eigen::P2F2P::position(double& distance)
{
    return {0,0};
}

double eigen::P2F2P::tangent_angle_rad(double& distance)
{
    return 0.;
}

double eigen::P2F2P::tangent_angle_deg(double& distance)
{
    return 0.;
}

double eigen::P2F2P::curvature(double& distance)
{
    return 0.;
}

double eigen::P2F2P::change_in_curvature(double& distance)
{
    return 0.;
}

void eigen::P2F2P::clear(void)
{
    this->waypoints_.clear();
    this->curve_.coefficients.clear();
    this->curve_.points.clear();
    this->curve_.order = 0;
    this->point_ = {0,0};
    this->frenet_ = {{0,0},{0,0}, 0, false, 0};
    return;
}

/**
 * Fits a polygon to the points obtained as input.
 * For now, a second order polynomial has been approximated
 * (Can be switched to higher order polynomial, or B-spline... if required)
 *
 * @param points List of way-points obtained as input
 * @return curve describing the coefficients of the polynomial
 */
sCurve eigen::P2F2P::obtain_curve(void)
{
    sCurve curve;
    std::vector<sCoeff> coeff;
    eigen::Compute::poly_fit_points(this->waypoints_, coeff, ORDER);
    curve.points = this->waypoints_;
    curve.order = ORDER;
    curve.coefficients = coeff;
    return curve;
}

/**
 * Find the waypoint nearest to the target
 *
 * @param curve Describes the coefficients of the polynomial
 * @param target The input point for which the Frenet frame is to be obtained
 * @return index of the point (among the input way points) which lies nearest to the target
 */
int eigen::P2F2P::find_nearest_point(const sPoint& target)
{
    int min_index = NEGATIVE_ONE;
    double min_dist = std::numeric_limits<double>::max();;
    double dist;
    double prev_dist;
    for(int i = 0; i < this->curve_.points.size(); i++)
    {
        dist = eigen::Compute::eucledian_distance(this->curve_.points[i], target);
        if(dist < min_dist)
        {
            min_index = i;
            min_dist = dist;
        }

        if((i > 0) && (dist > prev_dist))
        {
            return min_index;
        }
        prev_dist = dist;
    }
    return min_index;
}

/**
 * Find the closest distance of the target point from the curve
 *
 * @param curve Describes the coefficients of the polynomial
 * @param target The input point for which the Frenet frame is to be obtained
 * @param min_index Index of the point (among the input way points) which lies nearest to the target
 * @return frenet frame of the target point
 */
sFrenet eigen::P2F2P::distance_to_curve(const sPoint& target, int min_index)
{
    sFrenet frame = eigen::Compute::steepest_gradient_descent(this->curve_, target, min_index, ALPHA, MAXIMUM_ITERATION);
    return frame;
}


/**
 * Compute the distance of the closest point to the target from the origin, along the curve
 *
 * @param curve Describes the coefficients of the polynomial
 * @param frame Frenet frame of the target (the geodetic distance is updated here)
 * @param min_index Index of the point (among the input way points) which lies nearest to the target
 * @return void
 */
void eigen::P2F2P::geodetic_distance(sFrenet& frame, int min_index)
{
    int last_index = NEGATIVE_ONE;
    double dist_end = 0.0;
    bool Compute_distance = false;
    double cross_prod = 0.0;
    if((min_index >= 0) && (min_index < this->curve_.points.size()))
    {
        double dist_1 = eigen::Compute::eucledian_distance(this->curve_.points[min_index - 1], frame.cartesian_point);
        double dist_2 = eigen::Compute::eucledian_distance(this->curve_.points[min_index], frame.cartesian_point);
        if(dist_1 < dist_2)
        {
            last_index = min_index - 1;
            dist_end = dist_1;
            cross_prod = eigen::Compute::cross_product(this->curve_.points[min_index - 1], frame.cartesian_point);
        }
        else
        {
            last_index = min_index;
            dist_end = dist_2;
            cross_prod = eigen::Compute::cross_product(this->curve_.points[min_index], frame.cartesian_point);
        }
        Compute_distance = true;
    }
    else if(min_index == 0)
    {
        last_index = 0;
        frame.geodetic_distance = Compute::eucledian_distance(this->curve_.points[0],frame.cartesian_point);
        cross_prod = eigen::Compute::cross_product(this->curve_.points[0], frame.cartesian_point);
    }
    else
    {
        //we are doomed
        throw std::invalid_argument("Invalid index for point-on-curve closest to the target point. Can't proceed.");
    }
    if(Compute_distance)
    {
        double dist = 0.0f;

        for(int i = 1; i <= last_index; i++)
        {
            dist += Compute::eucledian_distance(this->curve_.points[i], this->curve_.points[i-1]);
        }
        dist+=dist_end;
        frame.geodetic_distance = dist;
    }
    if(cross_prod > 0)
    {
        frame.direction = true;
    }
    else
    {
        frame.direction = false;
    }
    return;
}

/* Extern */

std::ostream& eigen::operator<<(std::ostream& os, const P2F2P& o)
{
    os << "***** eigen object *****";
    os  << "\nTesting this" << std::endl;
    /* to do */
    return os;
}

/**
 * Begin spline spline spline spline spline spline spline spline spline spline spline spline spline spline spline 
 */

spline::P2F2P::P2F2P()
{
    LOG(Level::LINFO, "Init object without waypoints.");
}

spline::P2F2P::P2F2P(const std::vector<sPoint>& input)
{
    LOG(Level::LINFO, "Init object with point data.");
    if(input.size() > MAX_NUM_WAYPOINTS && input.size() < MIN_NUM_WAYPOINTS)
    {
        throw std::out_of_range("Reach max or min waypoints limit.");
    }
    this->init();
    this->points_ = input;
    LOG(Level::LINFO, "Input Points size: " + std::to_string(points_.size()));
    this->pre_calculator();
}

spline::P2F2P::~P2F2P()
{
    /* Clean up */
    this->init();
}

void spline::P2F2P::process_points(const std::vector<sPoint>& input)
{
    LOG(Level::LINFO, "Refresh object with point data.");
    if(input.size() > MAX_NUM_WAYPOINTS && input.size() < MIN_NUM_WAYPOINTS)
    {
        throw std::out_of_range("Reach max or min waypoints limit.");
    }
    this->init();
    this->points_ = input;
    this->pre_calculator();
    return;
}

sFrenet spline::P2F2P::g2f(const sPoint& point)
{
    if(PLOTS_ACTIVE_SPLINE)
    {
        system("start cmd /k python ../../plot/spline.py");
    }
    this->frenet_.cartesian_point = point;
    this->frenet_.closest_cartesian_point = this->closest_waypoint(point);
    this->frenet_.closest_point_on_curve = this->closest_point_on_curve(point);
    this->frenet_.geodetic_distance = this->geodetic_distance(this->frenet_.closest_point_on_curve);
    this->frenet_.lateral_distance = spline::Compute::eucledian_distance(this->frenet_.closest_point_on_curve, point);
    this->frenet_.direction = direction(point);
    LOG(Level::LINFO, "Point to frenet transformation complete.");
    return this->frenet_;
}

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
            target.x = this->sx_(t);
            target.y = this->sy_(t);
            LOG(Level::LINFO, "Found closest point on curve.");
            break;
        }
        last = actual;
    }
    LOG(Level::LINFO, "Method f2g loop counter: " + std::to_string(dummy));
    /* Find point in cartesian with lateral distance */
    /* Vector for direction */
    double dir_x = actual.x - last.x;
    double dir_y = actual.y - last.y;
    /* Normal vector */
    double normal_x;
    double normal_y;
    if (this->frenet_.direction) 
    {   /* 90 deg left */
        normal_x = -dir_y;
        normal_y = dir_x;
    } 
    else 
    {   /* 90 def right */
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
    LOG(Level::LINFO, "Cartesian Point found " + to_string(target));
    LOG(Level::LINFO, "Frenet to point transformation complete.");
    return target;
}

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
    LOG(Level::LINFO, "Path length calculated with " + std::to_string(path_length));
    return path_length;
}

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
            LOG(Level::LINFO, "Found point " + to_string(target) + " in " + std::to_string(distance));
            break;
        }
        last = actual;
    }
    LOG(Level::LINFO, "Position calculation finished.");
    return target;
}   

double spline::P2F2P::tangent_angle_deg(double& distance)
{
    double angle = this->tangent_angle_rad(distance) * (180 / M_PI);
    LOG(Level::LINFO, "Found point with with tangent angle " + std::to_string(angle) + " in deg");
    LOG(Level::LINFO, "Tangent angle calculation in deg finished.");
    return angle;
}

double spline::P2F2P::tangent_angle_rad(double& distance)
{
    /* Calculate derivative at distance */
    sPoint derv_point = {this->sx_.deriv(1, distance), this->sy_.deriv(1, distance)};
    LOG(Level::LINFO, "Deriv at distance target " + to_string(derv_point));
    /* Calculate angle */
    double angle = std::atan2(derv_point.y, derv_point.x);
    /* Convert to degree */
    LOG(Level::LINFO, "Found point with deriv " + to_string(derv_point) + " in " + std::to_string(distance) + 
            " with tangent angle " + std::to_string(angle) + " in rad");
    LOG(Level::LINFO, "Tangent angle calculation in rad finished.");
    return angle;
}

double spline::P2F2P::curvature(double& distance)
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
    LOG(Level::LINFO, "Calculated curvature at " + std::to_string(distance) +
            " is " + std::to_string(numerator / denominator));
    LOG(Level::LINFO, "Curvature calculation finished.");
    return numerator / denominator;
}

// double spline::P2F2P::change_in_curvature(double& distance)
// {
//     double kappa_1 = this->curvature(distance);   
//     double dis = distance + DISCRETIZATION_DISTANCE;        
//     double kappa_2 = this->curvature(dis);
//     double dkappa_ds = (kappa_2 - kappa_1) / DISCRETIZATION_DISTANCE;
//     LOG(Level::LINFO, "Calculated change in curvature at " + std::to_string(distance) +
//             " is " + std::to_string(dkappa_ds));
//     LOG(Level::LINFO, "Change in curvature calculation finished.");
//     return dkappa_ds;
// }

double spline::P2F2P::change_in_curvature(double& distance)
{
    double distance_plus = distance + DISCRETIZATION_DISTANCE;
    double distance_minus = distance - DISCRETIZATION_DISTANCE;
    double kappa_1 = this->curvature(distance_minus);
    double kappa_2 = this->curvature(distance_plus);
    double dkappa_ds = (kappa_2 - kappa_1) / (2 * DISCRETIZATION_DISTANCE);
    LOG(Level::LINFO, "Calculated change in curvature at " + std::to_string(distance) +
            " is " + std::to_string(dkappa_ds));
    LOG(Level::LINFO, "Change in curvature calculation finished.");
    return dkappa_ds;
}

void spline::P2F2P::pre_calculator(void)
{   
    LOG(Level::LINFO, "Enter pre calculation.");
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
    // LOG(Level::LINFO, "Input Pointssize from object: " + std::to_string(points_.size()));
    // LOG(Level::LINFO, "Extracted T's size: " + std::to_string(extract_T_.size())); 
    // LOG(Level::LINFO, "Extracted X's size: " + std::to_string(extract_X_.size())); 
    // LOG(Level::LINFO, "Extracted Y's size: " + std::to_string(extract_Y_.size())); 
    /* Spline interpolation for x(t) and y(t) */
    this->sx_.set_points(this->extract_T_, this->extract_X_);
    this->sy_.set_points(this->extract_T_, this->extract_Y_);
    LOG(Level::LINFO, "Spline interpolation complete.");
    /* Open file to write waypoints and the target point */
    std::ofstream file("../../plot/spline.txt");
    if (file.is_open() && PLOTS_ACTIVE_SPLINE) 
    {
        /* Ensure the first point is explicitly written as the starting point */
        // file << this->points_[0].x << "," << this->points_[0].y << std::endl;
        /* Iterate over the curve by stepping through the parameter 't' */
        for(double t = this->extract_T_.front(); t <= this->extract_T_.back(); t += DISCRETIZATION_DISTANCE)
        {
            /* Compute the coordinates of the point on the curve at parameter 't' */
            double x = this->sx_(t);
            double y = this->sy_(t);
            file << x << "," << y << std::endl;
        }
        file.close();
        LOG(Level::LINFO, "Spline successfully written to spline.txt");
    } 
    else if (PLOTS_ACTIVE_SPLINE == false)
    {
        LOG(Level::LWARNING, "Write spline in file and plot disabled.");
    }
    else 
    {
        LOG(Level::LERROR, "Unable to open file ../../plot/spline.txt for writing.");
    }
    LOG(Level::LINFO, "Run spline plot.");
    return;
}

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
    LOG(Level::LINFO, "Found closest waypoint.");
    return closest;
}

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
    LOG(Level::LINFO, "Found closest point on curve.");
    return closest;
}


double spline::P2F2P::geodetic_distance(const sPoint& target)
{
    double total_distance = 0;
    double min_distance_to_target = std::numeric_limits<double>::max();
    double geodetic_distance_to_target = 0;

    sPoint prev = {this->sx_(this->extract_T_.front()), this->sy_(this->extract_T_.front())};
    std::ofstream file("../../out/points.txt");
    double dummy = 0;
    if (file.is_open() && PLOTS_ACTIVE_POINTS) 
    {
        for(double t = this->extract_T_.front() + DISCRETIZATION_DISTANCE; 
                t <= this->extract_T_.back(); t += DISCRETIZATION_DISTANCE)
        {
            dummy++;
            sPoint actual = {this->sx_(t), this->sy_(t)};
            file << actual << std::endl;  
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
        file.close();
        LOG(Level::LINFO, "Waypoints and target point successfully written to file points.txt");
    } 
    else if(PLOTS_ACTIVE_POINTS == false)
    {
        LOG(Level::LWARNING, "Write points in out/points.txt disabled.");
    }
    else 
    {
        LOG(Level::LERROR, "Unable to open file ../../out/points.txt for writing.");
    }  
    LOG(Level::LINFO, "Geodetic distance counter: " + std::to_string(dummy));      
    LOG(Level::LINFO, "Geodetic distance calculated.");
    return geodetic_distance_to_target;
}

bool spline::P2F2P::direction(const sPoint& target)
{
    sPoint closest_on_curve = this->frenet_.closest_point_on_curve;
    double t_closest = this->sx_(closest_on_curve.x);
    double t_prev = t_closest - DISCRETIZATION_DISTANCE;
    sPoint prev_point = {this->sx_(t_prev), this->sy_(t_prev)};
    double cross_product = spline::Compute::cross_product(closest_on_curve, prev_point, target);
    LOG(Level::LINFO, "Found direction.");
    if (cross_product > 0) 
    {
        return false;  // Links
    } 
    else if (cross_product < 0)
    {
        return true;   // Rechts
    }
    else
    {
        throw std::runtime_error("Target on spline.");
    }
}


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