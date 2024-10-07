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
    //find the point among the input waypoints that lies closest to the input/target point
    int closest_point = find_nearest_point(target);            
    //find the closest point to the input/target point on the continuous curve and get the distance
    sFrenet frame = distance_to_curve(target, closest_point);   
    //find the distance to the closest point on the continuous curve from the curve's origin
    geodetic_distance(frame, closest_point);    
    this->frenet_ = frame;               
    return frame;                        
}

sPoint eigen::P2F2P::f2g(const sFrenet& target)
{
    sPoint point;
    return point;


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
    std::vector<double> coeff;
    NumMethods::poly_fit_points(this->waypoints_, coeff, ORDER);
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
    double min_dist = VERY_LARGE_NUMBER;
    double dist;
    double prev_dist;
    for(int i = 0; i < this->curve_.points.size(); i++)
    {
        dist = NumMethods::eucledian_distance(this->curve_.points[i], target);
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
    float alpha = ALPHA;
    int max_iteration = MAXIMUM_ITERATION;
    sFrenet frame = NumMethods::steepest_gradient_descent(this->curve_, target, min_index, alpha, max_iteration);
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
    double dist_end = 0.0f;
    bool compute_distance = false;
    float cross_prod;
    if((min_index >= 0) && (min_index < this->curve_.points.size()))
    {
        float dist_1 = NumMethods::eucledian_distance(this->curve_.points[min_index - 1], frame.cartesian_point);
        float dist_2 = NumMethods::eucledian_distance(this->curve_.points[min_index], frame.cartesian_point);
        if(dist_1 < dist_2)
        {
            last_index = min_index - 1;
            dist_end = dist_1;
            cross_prod = NumMethods::cross_product(this->curve_.points[min_index - 1], frame.cartesian_point);
        }
        else
        {
            last_index = min_index;
            dist_end = dist_2;
            cross_prod = NumMethods::cross_product(this->curve_.points[min_index], frame.cartesian_point);
        }
        compute_distance = true;
    }
    else if(min_index == 0)
    {
        last_index = 0;
        frame.geodetic_distance = NumMethods::eucledian_distance(this->curve_.points[0],frame.cartesian_point);
        cross_prod = NumMethods::cross_product(this->curve_.points[0], frame.cartesian_point);
    }
    else
    {
        //we are doomed
        throw std::invalid_argument("Invalid index for point-on-curve closest to the target point. Can't proceed.");
    }
    if(compute_distance)
    {
        double dist = 0.0f;

        for(int i = 1; i <= last_index; i++)
        {
            dist += NumMethods::eucledian_distance(this->curve_.points[i], this->curve_.points[i-1]);
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
}

/* Extern */

std::ostream& eigen::operator<<(std::ostream& os, const P2F2P& o)
{
    os << std::string(20, '*');
    os  << "\nPath length:" << std::endl;
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
    this->path_length_ = pre_calculator();
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
    this->path_length_ = pre_calculator();
    return;
}

sFrenet spline::P2F2P::g2f(const sPoint& point)
{
    /* Step 1: Find the closest point on the path and obtain the arc length (s) */
    this->point_ = point;
    this->frenet_ = closest_point(point);

    /* Step 2: Calculate the tangent vector */
    double dx_dt = this->sx_.deriv(1, this->frenet_.geodetic_distance);  // First derivative of x with respect to the arc length s
    double dy_dt = this->sy_.deriv(1, this->frenet_.geodetic_distance);  // First derivative of y with respect to the arc length s
    LOG(Level::LINFO, "Tangent vector: " + std::to_string(dx_dt) + "|" + std::to_string(dy_dt));

    /* Step 3: Calculate the normal vector (perpendicular to the tangent vector) */
    sPoint normal_vector = {-dy_dt, dx_dt};  // 90-degree rotation of the tangent vector

    /* Step 4: Calculate the length of the normal vector */
    double norm_len = std::sqrt(normal_vector.x * normal_vector.x + normal_vector.y * normal_vector.y);
    
    /* Ensure the normal vector is not too small */
    if (norm_len < 1e-6) 
    {
        LOG(Level::LERROR, "Normal vector length is too small, possible rounding error.");
        return this->frenet_;  // Handle error or return current Frenet state
    }

    /* Step 5: Normalize the normal vector */
    normal_vector.x /= norm_len;
    normal_vector.y /= norm_len;

    /* Step 6: Calculate the vector from the closest point on the path to the global point */
    sPoint diff = {point.x - this->frenet_.closest_point_on_curve.x, point.y - this->frenet_.closest_point_on_curve.y};

    /* Step 7: Project the difference vector onto the normalized normal vector to get lateral deviation (d) */
    this->frenet_.lateral_distance = (diff.x * normal_vector.x + diff.y * normal_vector.y);

    return this->frenet_;
}


sPoint spline::P2F2P::f2g(const sFrenet& frenet)
{
    /* Step 1: Calculate the point on the path corresponding to the arc length (s) */
    sPoint path_point = {this->sx_(frenet.geodetic_distance), this->sy_(frenet.geodetic_distance)};

    /* Step 2: Calculate the tangent vector (first derivative of the spline function) */
    double dx_dt = this->sx_.deriv(1, frenet.geodetic_distance);  // First derivative of x with respect to arc length s
    double dy_dt = this->sy_.deriv(1, frenet.geodetic_distance);  // First derivative of y with respect to arc length s
    LOG(Level::LINFO, "Tangent vector: " + std::to_string(dx_dt) + "|" + std::to_string(dy_dt));

    /* Step 3: Calculate the normal vector (perpendicular to the tangent vector, 90-degree rotation) */
    sPoint normal_vector = {-dy_dt, dx_dt};

    /* Step 4: Calculate the length of the normal vector */
    double norm_len = std::sqrt(normal_vector.x * normal_vector.x + normal_vector.y * normal_vector.y);

    /* Check if the normal vector length is too small to avoid division by zero */
    if (norm_len < 1e-6) 
    {
        LOG(Level::LERROR, "Normal vector length is too small, possible rounding error.");
        return path_point;  // Return the path point directly in case of rounding errors
    }

    /* Step 5: Normalize the normal vector */
    normal_vector.x /= norm_len;
    normal_vector.y /= norm_len;

    /* Step 6: Shift the point along the normal vector by the lateral deviation (d) */
    this->point_ = {path_point.x + frenet.lateral_distance * normal_vector.x,
                    path_point.y + frenet.lateral_distance * normal_vector.y};
                    
    return this->point_;
}

double spline::P2F2P::pre_calculator(void)
{   
    LOG(Level::LINFO, "Enter pre calculator.");
    /* Parameter calculator */
    double t = 0;
    // this->extract_T_.push_back(t);
    // this->extract_X_.push_back(0);
    // this->extract_Y_.push_back(0);
    for(size_t i = 1; i < this->points_.size(); i++)
    {
        t += euclidean_distance(this->points_[i], this->points_[i - 1]);
        this->extract_T_.push_back(t);
        this->extract_X_.push_back(this->points_[i].x);
        this->extract_Y_.push_back(this->points_[i].y);
    }
    LOG(Level::LINFO, "Input Pointssize from object: " + std::to_string(points_.size()));
    LOG(Level::LINFO, "Extracted T's size: " + std::to_string(extract_T_.size()));
    LOG(Level::LINFO, "Extracted X's size: " + std::to_string(extract_X_.size()));
    LOG(Level::LINFO, "Extracted Y's size: " + std::to_string(extract_Y_.size()));
    /* Spline interpolation for x(t) and y(t) */
    this->sx_.set_points(this->extract_T_, this->extract_X_);
    this->sy_.set_points(this->extract_T_, this->extract_Y_);
    /* Calculate length interpolated path with numerical intergration */
    double path_length = 0;
    double prev_x = this->points_[0].x;
    double prev_y = this->points_[0].y;
    for(double t = this->extract_T_.front(); t <= this->extract_T_.back(); t += DISCRETIZATION_DISTANCE)
    {
        double x = this->sx_(t);
        double y = this->sy_(t);
        path_length += euclidean_distance({prev_x, prev_y}, {x, y});
        prev_x = x;
        prev_y = y;
    }
    return path_length;
}

double spline::P2F2P::euclidean_distance(const sPoint& p1, const sPoint& p2)
{
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

void spline::P2F2P::init(void)
{
    this->points_.clear();
    this->frenet_.cartesian_point = {0.0,0.0};
    this->frenet_.closest_point_on_curve = {0.0,0.0};
    this->frenet_.geodetic_distance = 0.0;
    this->frenet_.lateral_distance = 0.0;
    this->point_ = {0.0,0.0};
    this->path_length_ = 0.0;
    this->extract_X_.clear();
    this->extract_Y_.clear();
    this->extract_T_.clear();
    return;
}

sFrenet spline::P2F2P::closest_point(const sPoint& point)
{
    sFrenet frenet;
    double min_dist = std::numeric_limits<double>::max();
    double closest_s = 0;
    /* Find next point on path */
    for(double t = this->extract_T_.front(); t <= this->extract_T_.back(); t += DISCRETIZATION_DISTANCE)
    {
        sPoint path_point = {this->sx_(t), this->sy_(t)};
        double dist = euclidean_distance(path_point, point);
        if(dist < min_dist)
        {
            min_dist = dist;
            closest_s = t;
        }
    }
    frenet.geodetic_distance = closest_s;
    frenet.closest_point_on_curve = {this->sx_(closest_s), this->sy_(closest_s)};
    return frenet;
}

/* Extern */

std::ostream& spline::operator<<(std::ostream& os, const P2F2P& o)
{
    os << std::string(20, '*');
    os  << "\nPath length:                    " << o.path_length_ << "\n"
        << "Discretization distance:        " << DISCRETIZATION_DISTANCE << "\n"
        << "Waypoints:                      " << o.points_.size() << "\n"
        << "Max waypoints                   " << MAX_NUM_WAYPOINTS << "\n"
        << "Frenet:   *Closest point:       " << o.frenet_.closest_point_on_curve.x << "|" << o.frenet_.closest_point_on_curve.y << "\n"
        << "          *Arc length:          " << o.frenet_.geodetic_distance << "\n"
        << "          *Lateral deviation:   " << o.frenet_.lateral_distance << "\n"
        << "Point:    *x:                   " << o.point_.x << "\n"
        << "          *y:                   " << o.point_.y << "\n";
    return os;
}

/* Eof */