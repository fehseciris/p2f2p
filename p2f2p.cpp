#include "p2f2p.h"

P2F2P::P2F2P(/* Default init */)
{
    LOG(Level::LINFO, "Init object without data.");
}

P2F2P::P2F2P(std::vector<sFrenet> input)
{
    LOG(Level::LINFO, "Init object with frenet data.");
    throw std::runtime_error("Frenet input not implemented.");
    /* To do */
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
        throw std::runtime_error("Frenet input not implemented.");
        /* To do */
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

void P2F2P::curvature()
{
    return;
}

void P2F2P::change_in_curvature()
{
    return;
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

void P2F2P::position()
{
    return;
}

void P2F2P::tangentAngle()
{
    return;
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

void P2F2P::upload(std::vector<sFrenet> input)
{
    LOG(Level::LINFO, "Refresh object with frenet data.");
    throw std::runtime_error("Frenet refresh not implemented.");
    /* To do */
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