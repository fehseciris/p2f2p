# p2f2p
This project converts points from cartesian coordinates to its equivalent representation in frenet coordinates or otherwise.

## Build options
Build with cmake extension in vs code. Maybe this works eather on linux systems. (GCC 13.1.0 x86_64-w64-mingw32)

Use the microsoft visual studio solution to build with MVSC directily in vs.

## Functionality
All functionalities are described by the IP2F2P interface. The derived classes implement these methods with different numerical calculations. Another description in file "Ip2f2p.h".

Insert new waypoints in P2F2P object. Also possible with constructor while create this.
-> void process_points(const std::vector<sPoint>\&)

Convert global states to frenet states and return.
-> sFrenet g2f(const sPoint\&)

Convert frenet states to global states and return.
-> sPoint f2g(const sFrenet\&)

Return total arclength along the path.
-> double path_length(void)

Return x,y position at arclength.
-> sPoint position(double\&)

Return tangent angle at arclength.
-> double tanget_angle(double\&)

Return curvature at arclength.
-> void curvature(void)

Return change-in-curvature at arclength.
-> void changeln_curvature(void)