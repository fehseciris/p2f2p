#include "p2f2p.h"

P2F2P::P2F2P(/* Default init */)
{
    LOG(Level::LINFO, "Init object without data.");
}

P2F2P::P2F2P(const std::vector<sPoint>& input)
{
    LOG(Level::LINFO, "Init object with point data.");
    if(input.size() > MAX_NUM_WAYPOINTS)
    {
        throw std::out_of_range("Reach max waypoints limit.");
    }
    this->points_ = pre_calculator(input);
}

P2F2P::~P2F2P()
{
    /* Clean up */
    this->points_.clear();
}

void P2F2P::upload(const std::vector<sPoint>& input)
{
    LOG(Level::LINFO, "Refresh object with point data.");
    if(input.size() > MAX_NUM_WAYPOINTS)
    {
        throw std::out_of_range("Reach max waypoints limit.");
    }
    /* Clean up */
    this->points_ = pre_calculator(input);
    return;
}

std::vector<sPoint> P2F2P::collect(int argc, char* argv[])
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

std::vector<sPoint> P2F2P::pre_calculator(const std::vector<sPoint>& input)
{
    /* Clean up */
    this->points_.clear();
    this->extract_X_.clear();
    this->extract_Y_.clear();
    this->path_length_ = 0;
    /* Extract */
    for(const auto& point : input)
    {
        extract_X_.push_back(point.x);
        extract_Y_.push_back(point.y);
    }
    tk::spline spline;
    spline.set_points(extract_X_, extract_Y_);
    /* Numerical integration */
    double path_length = 0;
    double prev_X = extract_X_[0];
    double prev_Y = extract_Y_[0];
    /* Loop */
    for (double x = extract_X_[0]; x <= extract_X_.back(); x += DISCRETIZATION_DISTANCE) 
    {
        double y = spline(x);  
        double dx = x - prev_X;
        double dy = y - prev_Y;
        path_length += std::sqrt(dx * dx + dy * dy);
        prev_X = x;
        prev_Y = y;
    }
    this->path_length_ = path_length;
    return input;
}

/* Extern */

std::ostream& operator<<(std::ostream& os, const P2F2P& o)
{
    os  << "Path length:                " << o.path_length_ << "\n"
        << "Discretization distance:    " << DISCRETIZATION_DISTANCE << "\n"
        << "Waypoints:                  " << o.points_.size() << "\n"
        << "Max waypoints               " << MAX_NUM_WAYPOINTS << "\n";
    return os;
}

/* Eof */