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
 * @brief Compares two sPoint objects for equality.
 * This operator checks if the x and y coordinates of two sPoint objects are equal.
 * @param a The first sPoint object to compare.
 * @param b The second sPoint object to compare.
 * @return true if both the x and y coordinates of the two sPoint objects are equal, false otherwise.
 */
bool operator==(const sPoint& a, const sPoint& b)
{
    return ((a.x == b.x) && (a.y == b.y));
}

/**
 * Converts an sPoint object to a string in the format "P(x|y)".
 * @param point The sPoint object containing x and y coordinates.
 * @return A string representation of the point in the format "P(x|y)".
 */
std::string pto_string(const sPoint& point)
{
    std::string str = {"P("};
    str += std::to_string(point.x);
    str += "|";
    str += std::to_string(point.y);
    str += ")";
    return str;
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

bool operator==(const sFrenet& a, const sFrenet& b)
{
    return ((a.cartesian_point == b.cartesian_point) && 
            (a.closest_cartesian_point == b.closest_cartesian_point) && 
            (a.closest_point_on_curve == b.closest_point_on_curve) && 
            (a.geodetic_distance == b.geodetic_distance) && 
            (a.lateral_distance == b.lateral_distance) && 
            (a.direction == b.direction));
}

/**
 * Checks if a file exists at the given path.
 * Uses the filesystem library to check for file existence.
 * @param path The path to the file to be checked.
 * @return True if the file exists at the specified path, false otherwise.
 */
bool file_exists(const std::string& path) 
{
    if (std::filesystem::exists(path))
    {
        LOG(Level::LDEBUG, Topic::P2F2P, "Filepath " + path + " exists");
    }
    return std::filesystem::exists(path);
}

/**
 * Collects and parses command-line arguments to create a vector of sPoint objects.
 * This function interprets command-line arguments in pairs to form points (x, y).
 * Only works if the first argument is "-p", followed by an even number of additional arguments.
 * Throws exceptions for invalid argument counts or non-numeric values.
 * @param argc The count of command-line arguments.
 * @param argv The array of command-line argument strings.
 * @return A vector of sPoint objects, each representing a point with x and y coordinates.
 * @throws std::invalid_argument If an incorrect number of arguments is provided, 
 *         if arguments are non-numeric, or if the first argument is not "-p".
 * @throws std::out_of_range If a numeric argument exceeds the allowable range.
 * @throws std::runtime_error If no arguments are provided (argc <= 1).
 */
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
                catch (const std::invalid_argument&) 
                {
                    throw std::invalid_argument("Invalid argument: non-numeric value.");
                } 
                catch (const std::out_of_range&) 
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
 * Writes the waypoints and the target point to a file for plotting.
 * @param filename The name of the file to write to.
 * @param points
 * @param target
 */
void points_to_file(const std::string& filename, 
        const std::vector<sPoint>& points,
        const sPoint& target)
{
    /* Check if active */
    if(!PYTHON_PLOTS)
    {
        LOG(Level::LDEBUG, Topic::P2F2P, "Python plot disabled (points_to_file)");
        return;
    }
    std::ofstream file(filename);
    if (file.is_open()) 
    {
        /* Write points in file */
        for (const auto& point : points) 
        {
            file << point.x << "," << point.y << std::endl;  
        }
        /* Check target */
        if(target.x != 0 && target.y != 0)
        {
            file << "target," << target.x << "," << target.y << std::endl;
        }
        file.close();
        LOG(Level::LDEBUG, Topic::P2F2P, "Waypoints and target point successfully written to " + filename);
        return;
    } 
    else
    {
        LOG(Level::LDEBUG, Topic::P2F2P, "Unable to open file " + filename);
        return;
    }
}

/* Eof */