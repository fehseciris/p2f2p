#include "p2f2p.h"

P2F2P::P2F2P(/* Default init */)
{
    LOG(Level::LINFO, "Init object without data.");
}

P2F2P::P2F2P(std::vector<sFrenet> input)
{
    LOG(Level::LINFO, "Init object with frenet data.");
    this->frenets_ = input;
}

P2F2P::P2F2P(std::vector<sPoint> input)
{
    LOG(Level::LINFO, "Init object with point data.");
    this->points_ = p2f2p::Core::expand(input);
}

P2F2P::P2F2P(VariantVector variant)
{
    if(std::holds_alternative<std::vector<sFrenet>>(variant))
    {
        LOG(Level::LINFO, "Init object with point data from parser.");
        this->frenets_ = std::get<std::vector<sFrenet>>(variant);
    }
    if(std::holds_alternative<std::vector<sPoint>>(variant))
    {
        LOG(Level::LINFO, "Init object with frenet data from parser.");
        this->points_ = p2f2p::Core::expand(std::get<std::vector<sPoint>>(variant));
    }
}

P2F2P::~P2F2P()
{
    /* Clean up */
    this->frenets_.clear();
    this->points_.clear();
}

/**
 * ### Matlab object functions
 */

void P2F2P::closest_point()
{
    throw std::runtime_error("Function body empty.");
    return;
}

void P2F2P::closest_points_to_sequence()
{
    throw std::runtime_error("Function body empty.");
    return;
}

void P2F2P::closest_projections()
{
    throw std::runtime_error("Function body empty.");
    return;
}

double P2F2P::curvature(double s)
{
    /* Check if points_ is empty */
    if (this->points_.empty()) 
    {
        throw std::runtime_error("No points available to calculate curvature.");
    }
    /* Initialize the closest point */
    double min_distance = std::numeric_limits<double>::max();
    double closest_kappa = 0.0;
    /* Iterate over the points to find the closest s */
    for (const auto& point : this->points_) 
    {
        double distance = std::abs(point.s - s);
        /* If this point is closer to the given s, update the closest kappa */
        if (distance < min_distance) 
        {
            min_distance = distance;
            closest_kappa = point.kappa;
        }
    }
    /* Return the curvature (kappa) of the closest point */
    return closest_kappa;
}

void P2F2P::change_in_curvature()
{
    /* Check if there are enough points to calculate curvature change */
    if (this->frenets_.size() < 2 || this->points_.size() < 2) 
    {
        throw std::runtime_error("Not enough points to calculate curvature change.");
    }
    /* Check if the sizes of frenets_ and points_ are consistent */
    if (this->frenets_.size() != this->points_.size())
    {
        throw std::runtime_error("Invalid data size.");
    }
    for (size_t i = 1; i < frenets_.size(); ++i) 
    {
        /* Calculate the difference in arc length and curvature */
        double delta_s = this->frenets_[i].geodetic_distance - frenets_[i - 1].geodetic_distance;
        double delta_kappa = this->points_[i].kappa - this->points_[i - 1].kappa;
        if (delta_s != 0) 
        {
            /* Calculate the derivative of curvature (dkappa) */
            frenets_[i].lateral_distance = delta_kappa / delta_s;
        } 
        else 
        {
            /* No change in s means no change in kappa */
            frenets_[i].lateral_distance = 0.0;  
        }
        /* Optional: Update the corresponding sAPoint with the new dkappa */
        this->points_[i].dkappa = frenets_[i].lateral_distance;
    }
}



void P2F2P::frenet2global()
{
    if(this->frenets_.empty())
    {
        throw std::runtime_error("Data frenets_ empty.");
    }
    return;
}

void P2F2P::global2frenet()
{
    if(this->points_.empty())
    {
        throw std::runtime_error("Data points_ empty.");
    }
    /* Process points */
    std::shared_ptr<p2f2p::Core> core = std::make_shared<p2f2p::Core>();
    /* Workbench */
    this->frenets_ = core->g2f(this->points_);
    return;
}

void P2F2P::interpolate()
{
    throw std::runtime_error("Function body empty.");
    return;
}

sPoint P2F2P::position(double s)
{
    /* Check if points_ is empty */
    if (this->points_.empty()) 
    {
        throw std::runtime_error("No points available to calculate position.");
    }
    /* Initialize the closest point */
    double min_distance = std::numeric_limits<double>::max();
    sPoint closest_point = {0.0, 0.0};
    /* Iterate over the points to find the closest s */
    for (const auto& point : this->points_) 
    {
        double distance = std::abs(point.s - s);
        /* If this point is closer to the given s, update the closest point */
        if (distance < min_distance) 
        {
            min_distance = distance;
            closest_point = point.cartesian_point;
        }
    }
    /* Return the cartesian position (x, y) of the closest point */
    return closest_point;
}

double P2F2P::tangentAngle(double s)
{
    /* Check if points_ is empty */
    if (this->points_.empty()) 
    {
        throw std::runtime_error("No points available to calculate tanget angle.");
    }
    /* Initialize the closest point */
    double min_distance = std::numeric_limits<double>::max();
    double closest_theta = 0.0;
    /* Iterate over the points to find the closest s */
    for (const auto& point : this->points_) 
    {
        double distance = std::abs(point.s - s);
        /* If this point is closer to the given s, update the closest theta */
        if (distance < min_distance) 
        {
            min_distance = distance;
            closest_theta = point.theta;
        }
    }
    /* Return the tangent angle (theta) of the closest point */
    return closest_theta;
}

void P2F2P::show()
{
    throw std::runtime_error("Function body empty.");
    return;
}

void P2F2P::copy() 
{
    throw std::runtime_error("Function body empty.");
    return;
}

/**
 * ### Customised functions 
 */

std::vector<sAPoint> P2F2P::get_points()
{
    return this->points_;
}

std::vector<sFrenet> P2F2P::get_frenets()
{
    return this->frenets_;
}

double P2F2P::path_length()
{
    return this->points_.back().s;
}

void P2F2P::upload(std::vector<sFrenet> input)
{
    LOG(Level::LINFO, "Refresh object with frenet data.");
    /* Clean up */
    this->frenets_.clear();
    this->points_.clear();
    this->frenets_ = input;
    return;
}

void P2F2P::upload(std::vector<sPoint> input)
{
    LOG(Level::LINFO, "Refresh object with point data.");
    /* Clean up */
    this->points_.clear();
    this->frenets_.clear();
    this->points_ = p2f2p::Core::expand(input);
    return;
}

P2F2P::VariantVector P2F2P::collect(int argc, char * argv[])
{
    if(argc > 1)
    {
        if (std::string(argv[1]) == "-p") 
        {
            if((argc - 1) % 2 != 0)
            {
                throw std::invalid_argument("Invalid amount of arguments.");
            }
            std::vector<sPoint> result;
            sPoint dummy;
            for (int i = 2; i < argc; i += 2) 
            {
                dummy.x = std::stoi(argv[i]);
                dummy.y = std::stoi(argv[i + 1]);
                result.push_back(dummy);
            }
            return result;
        } 
        if (std::string(argv[1]) == "-f")
        {
            if((argc - 1) % 5 != 0)
            {
                throw std::invalid_argument("Invalid amount of arguments.");
            }
            else
            {
                /* To do */
                throw std::runtime_error("Frenet input not implemented.");
            }
            std::vector<sFrenet> result;
            for (int i = 2; i < argc; i++) 
            {
                
            }
            return result;
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

/* Eof */