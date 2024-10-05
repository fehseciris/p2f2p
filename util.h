#pragma once

#define PRECISION setprecision(10)

#define RESET               "\033[0m"
#define BLACK               "\033[30m"              /* Black */
#define RED                 "\033[31m"              /* Red */
#define GREEN               "\033[32m"              /* Green */
#define YELLOW              "\033[33m"              /* Yellow */
#define BLUE                "\033[34m"              /* Blue */
#define MAGENTA             "\033[35m"              /* Magenta */
#define CYAN                "\033[36m"              /* Cyan */
#define WHITE               "\033[37m"              /* White */
#define BOLDBLACK           "\033[1m\033[30m"       /* Bold Black */
#define BOLDRED             "\033[1m\033[31m"       /* Bold Red */
#define BOLDGREEN           "\033[1m\033[32m"       /* Bold Green */
#define BOLDYELLOW          "\033[1m\033[33m"       /* Bold Yellow */
#define BOLDBLUE            "\033[1m\033[34m"       /* Bold Blue */
#define BOLDMAGENTA         "\033[1m\033[35m"       /* Bold Magenta */
#define BOLDCYAN            "\033[1m\033[36m"       /* Bold Cyan */
#define BOLDWHITE           "\033[1m\033[37m"       /* Bold White */

// #include <list>
// #include <iostream>
// #include <sstream>
// #include <vector>
// #include <cstring>
// #include <algorithm>
// #include <string>
// #include <iostream>
// #include <fstream>
// #include <stdio.h>
// #include <iomanip>


#define VERY_LARGE_NUMBER       999999.99f
#define NEGATIVE_ONE            -1

#define ORDER                   2
#define TERMINATION_CRITERIA    0.0001f
#define ALPHA                   0.5f
#define MAXIMUM_ITERATION       25

struct sPoint
{
    double x;
    double y;
};

struct sInput
{
    std::vector<point> points;
    point target;
};

struct sCurve
{
    int order;
    std::vector<double> coefficients;
    std::vector<point> points;
};

struct sFrenet
{
    sPoint cartesian_point;
    sPoint closest_point_on_curve;
    double geodetic_distance;
    bool direction; //true -> left; false -> right
    double lateral_distance;
};

/* Eof */