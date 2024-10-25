#include "util.h"

/**
 * Overloaded operator<< to output an sPoint object to an output stream.
 * Formats the point as "P(x|y)".
 * @param os The output stream to which the point will be written.
 * @param point The sPoint object containing x and y coordinates.
 * @return The modified output stream.
 */
std::ostream& operator<<(std::ostream& os, const sPoint& point)
{
    os << "P(" << point.x << "|" << point.y << ")";
    return os;
}

/**
 * Overloaded operator<< to output a vector of sPoint objects to an output stream.
 * Formats each point in the vector as "P(x|y) - " and separates each point with " - ".
 * @param os The output stream to which the vector of points will be written.
 * @param points The vector of sPoint objects containing x and y coordinates.
 * @return The modified output stream.
 */
std::ostream& operator<<(std::ostream& os, const std::vector<sPoint>& points)
{
    for(auto& it : points)
    {
        os << it << " - ";
    }
    os << "\n";
    return os;
}

/**
 * Converts an sPoint object to a string in the format "P(x|y)".
 * @param point The sPoint object containing x and y coordinates.
 * @return A string representation of the point in the format "P(x|y)".
 */
std::string to_string(const sPoint& point)
{
    std::string str = {"P("};
    str += std::to_string(point.x);
    str += "|";
    str += std::to_string(point.y);
    str += ")";
    return str;
}

/**
 * Overloaded operator<< to output an sCoeff object to an output stream.
 * Formats the coefficients as "C(a0: value | a1: value | a2: value | a3: value)".
 * @param os The output stream to which the coefficients will be written.
 * @param coeff The sCoeff object containing the polynomial coefficients a0, a1, a2, and a3.
 * @return The modified output stream.
 */
std::ostream& operator<<(std::ostream& os, const sCoeff& coeff)
{
    os << "C(a0: " 
        << coeff.a0 << " | a1: "
        << coeff.a1 << " | a2: "
        << coeff.a2 << " | a3: "
        << coeff.a3 << ")\n";
    return os;
}

/**
 * Overloaded operator<< to output an sFrenet object to an output stream.
 * Formats the frenet properties in a structured format with labels for each property.
 * @param os The output stream to which the Frenet object will be written.
 * @param frenet The sFrenet object containing the Frenet properties, including cartesian point,
 *               closest cartesian point, closest point on curve, geodetic distance,
 *               lateral distance, and direction (left or right).
 * @return The modified output stream.
 */
std::ostream& operator<<(std::ostream& os, const sFrenet& frenet)
{
    std::string str = {""};
    if(frenet.direction == true)
    {
        str = "left";
    }
    else
    {
        str = "right";
    }
    os << "F{Cartesian " << frenet.cartesian_point 
        << " - CloseK " << frenet.closest_cartesian_point
        << " - ClosePoC " << frenet.closest_point_on_curve
        << " - GeodeticD " << frenet.geodetic_distance
        << " - LateralD " << frenet.lateral_distance
        << " - Direction " << str
        << "}\n";
    return os;
}

/**
 * Checks if a file exists at the given path.
 * Uses the filesystem library to check for file existence.
 * @param path The path to the file to be checked.
 * @return True if the file exists at the specified path, false otherwise.
 */
bool file_exists(const std::string& path) 
{
    return std::filesystem::exists(path);
}

/* Eof */