#pragma once

#include <iostream>
#include <vector>
#include <exception>
#include <variant>
#include <memory>

#include "Ip2f2p.h"
#include "util.h"
#include "num.h"
#include "iol.h"

#include "../spline/src/spline.h"

static std::vector<sPoint> collect(int argv, char* argc[]);

namespace eigen
{
    class P2F2P : public Ip2f2p
    {
    public:
        P2F2P();
        P2F2P(const std::vector<sPoint>& points);
        ~P2F2P();
        /* Explicit delete */
        P2F2P(const P2F2P&) = delete;
        P2F2P& operator=(const P2F2P&) = delete;
        P2F2P(P2F2P&&) = delete;
        P2F2P& operator=(P2F2P&&) = delete;

        void process_points(const std::vector<sPoint>& points) override;
        sFrenet g2f(const sPoint& target) override;
        sPoint f2g(const sFrenet& target) override;
        double path_length(void) override;
        sPoint position(double& distance) override;
        double tangent_angle(double& distance) override;

        friend std::ostream& operator<<(std::ostream& os, const P2F2P& o);

    private:
        void clear(void);
        sCurve obtain_curve(void);
        int find_nearest_point(const sPoint& target);
        sFrenet distance_to_curve(const sPoint& target, int min_index);
        void geodetic_distance(sFrenet& frame, int min_index);

        /* Members */
        std::vector<sPoint> waypoints_;
        sCurve curve_;
        sPoint point_;
        sFrenet frenet_;

    };

    std::ostream& operator<<(std::ostream& os, const P2F2P& o);
};

namespace spline
{
    class P2F2P : public Ip2f2p
    {
    public:
        P2F2P();
        P2F2P(const std::vector<sPoint>& points);
        ~P2F2P();
        /* Explicit delete */
        P2F2P(const P2F2P&) = delete;
        P2F2P& operator=(const P2F2P&) = delete;
        P2F2P(P2F2P&&) = delete;
        P2F2P& operator=(P2F2P&&) = delete;

        void process_points(const std::vector<sPoint>& points) override;
        sFrenet g2f(const sPoint& target) override;
        sPoint f2g(const sFrenet& target) override;
        double path_length(void) override;
        sPoint position(double& distance) override;
        double tangent_angle(double& distance) override;

        friend std::ostream& operator<<(std::ostream& os, const P2F2P& o);

    private:
        void pre_calculator(void);
        sPoint closest_waypoint(const sPoint& target);
        sPoint closest_point_on_curve(const sPoint& target);
        double geodetic_distance(const sPoint& target);
        bool direction(const sPoint& target);
        void init(void);
        

        /* Members */
        std::vector<sPoint> points_;
        sFrenet frenet_;
        sPoint point_;
        sPoint target_;

        tk::spline sx_;
        tk::spline sy_;
        std::vector<double> extract_X_; // x points 
        std::vector<double> extract_Y_; // y points 
        std::vector<double> extract_T_; // Add eucledian distance
        std::vector<double> extract_D_; // Distance between points 

    };

    std::ostream& operator<<(std::ostream& os, const P2F2P& o);
};

/* Eof */