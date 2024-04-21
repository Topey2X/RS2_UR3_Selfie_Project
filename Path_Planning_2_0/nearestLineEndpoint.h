#ifndef NEARESTLINEENDPOINT_H
#define NEARESTLINEENDPOINT_H

#include <vector>
#include <cmath>
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <fstream>

struct Point {
    double x;
    double y;
    double z;
};

double calculateDistance(const Point& p1, const Point& p2);
int findNearestLine(const Point& currentPoint, const std::vector<std::vector<Point>>& lines, const std::vector<bool>& visitedLines);
double calculateTotalDistance(const std::vector<std::vector<Point>>& path);
std::vector<std::vector<Point>> optimizePathWithLiftoff(const std::vector<std::vector<Point>>& lines);
#endif