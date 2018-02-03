

#ifndef PATH_PLANNING_HELPER_H
#define PATH_PLANNING_HELPER_H

#include <vector>

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

constexpr double MPS_TO_MILESPH = 2.237;
constexpr double TIME_INTERVAL = 0.02;

#endif //PATH_PLANNING_HELPER_H
