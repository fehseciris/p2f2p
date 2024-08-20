#include "core.h"

using namespace p2f2p;

Core::Core()
{

}

Core::~Core()
{

}

std::vector<sFrenet> Core::g2f(const std::vector<sAPoint>& waypoints)
{
    std::vector<sFrenet> frenets;
    for (const auto& point : waypoints) 
    {
        sFrenet frenet;
        frenet.cartesian_point = point.cartesian_point;

        double min_distance = std::numeric_limits<double>::max();
        sPoint closest_point = { 0,0 };
        double s_value = 0.0;

        for (size_t i = 0; i < waypoints.size() - 1; ++i) 
        {
            /* Line between two points */
            const auto& p1 = waypoints[i];
            const auto& p2 = waypoints[i + 1];

            double dx = p2.cartesian_point.x - p1.cartesian_point.x;
            double dy = p2.cartesian_point.y - p1.cartesian_point.y;
            double length = std::sqrt(dx * dx + dy * dy);

            /* Parametric representation of the point on the line */
            double t = ((point.cartesian_point.x - p1.cartesian_point.x) 
                    * dx + (point.cartesian_point.y - p1.cartesian_point.y) * dy) / (length * length);

            /* Clamping t to the range [0, 1]  */
            t = std::max(0.0, std::min(1.0, t));

            /* Projection of the point onto the line */
            sPoint projection;
            projection.x = p1.cartesian_point.x + t * dx;
            projection.y = p1.cartesian_point.y + t * dy;

            /* Distance from the point to the projection */
            double distance = std::sqrt(std::pow(projection.x - point.cartesian_point.x, 2) + 
                    std::pow(projection.y - point.cartesian_point.y, 2));

            /* Calculate arc length s */
            double s_segment = std::sqrt(std::pow(projection.x - p1.cartesian_point.x, 2) + 
                    std::pow(projection.y - p1.cartesian_point.y, 2));

            if (distance < min_distance) 
            {
                min_distance = distance;
                closest_point = projection;
                s_value = s_segment;
            }
        }

        frenet.closest_point_on_curve = closest_point;
        frenet.geodetic_distance = s_value;
        frenet.lateral_distance = min_distance;

        /* Determine the direction (left or right from the path) */
        double cross_product = (point.cartesian_point.x - closest_point.x) * 
                (closest_point.y - point.cartesian_point.y) - (point.cartesian_point.y - closest_point.y) * 
                (closest_point.x - point.cartesian_point.x);
        frenet.direction = (cross_product > 0);

        frenets.push_back(frenet);
    }
    return frenets;
}

std::vector<sAPoint> Core::f2g(const std::vector<sFrenet>& frenets)
{
    std::vector<sAPoint> aPoints;
    for (const auto& frenet : frenets) 
    {
        sAPoint apoint;
        /* Assign Cartesian coordinates */
        apoint.cartesian_point = frenet.cartesian_point;

        /* Assign arc length s from Frenet coordinates */
        apoint.s = frenet.geodetic_distance;

        /* Calculate theta (orientation) based on Frenet coordinates */
        double dx = frenet.cartesian_point.x - frenet.closest_point_on_curve.x;
        double dy = frenet.cartesian_point.y - frenet.closest_point_on_curve.y;
        apoint.theta = std::atan2(dy, dx);
        /**
         * Calculate kappa (curvature)
         * Since kappa represents the curvature of the path, we assume it to be
         * the curvature of the line segment corresponding to the Frenet coordinates.
         * If kappa is known, it can be used directly.
         * Here we assume kappa = 1/R, where R is the distance between the point and
         * the closest point on the curve.
         */
        double R = std::sqrt(dx * dx + dy * dy);
        apoint.kappa = (R != 0) ? (1.0 / R) : 0.0;
        /**
         * dkappa represents the change in curvature along the curve, and without
         * additional information, it cannot be calculated, so we set it to 0.0.
         */
        apoint.dkappa = 0.0;

        /* Add the calculated sAPoint to the vector */
        aPoints.push_back(apoint);
    }

    return aPoints;
}

std::vector<sAPoint> Core::expand(std::vector<sPoint> waypoints)
{
    double total_s = 0.0;
    std::vector<sAPoint> path;
    for (size_t i = 0; i < waypoints.size(); ++i) 
    {
        sAPoint point;
        /* Set the position */
        point.cartesian_point.x = waypoints[i].x;
        point.cartesian_point.y = waypoints[i].y;
        if (i > 0) 
        {
            /* Calculate s (arc length) */
            double dx = point.cartesian_point.x - waypoints[i - 1].x;
            double dy = point.cartesian_point.y - waypoints[i - 1].y;
            double ds = std::sqrt(dx * dx + dy * dy);
            total_s += ds;
            point.s = total_s;
            /* Calculate theta (orientation) */
            point.theta = std::atan2(dy, dx);
            /* Calculate kappa (curvature) and dkappa (rate of change of curvature) */
            if (i > 1) 
            {
                double prev_dx = waypoints[i - 1].x - waypoints[i - 2].x;
                double prev_dy = waypoints[i - 1].y - waypoints[i - 2].y;
                double prev_theta = std::atan2(prev_dy, prev_dx);
                double dtheta = point.theta - prev_theta;
                point.kappa = dtheta / ds;
                double prev_kappa = path[i - 1].kappa;
                point.dkappa = (point.kappa - prev_kappa) / ds;
            } 
            else 
            {
                point.kappa = 0.0;
                point.dkappa = 0.0;
            }
        } 
        else 
        {
            /* For the first point */
            point.s = 0.0;
            point.theta = 0.0;
            point.kappa = 0.0;
            point.dkappa = 0.0;
        }
        path.push_back(point);
    }
    return path;
}

/* Eof */
