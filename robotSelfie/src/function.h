#ifndef FUNCTIONROS_H
#define FUNCTIONROS_H

#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <fstream>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point32.h>

struct Point {
    double x;
    double y;
    double z;
};

double calculateDistance(const Point& p1, const Point& p2);
int findNearestLine(const Point& currentPoint, const std::vector<std::vector<Point>>& lines, const std::vector<bool>& visitedLines);
double calculateTotalDistance(const std::vector<std::vector<Point>>& path);
// std::vector<std::vector<Point>> convertToMM(const std::vector<std::vector<geometry_msgs::Point>>& lines, double pixelsPerMM, double offsetX, double offsetY);
nav_msgs::Path optimizePathWithLiftoff(const std::vector<std::vector<Point>>& lines, bool printDistances = false);
void writeToCsv(const nav_msgs::Path& optimizedPath, const std::string& filename);
#endif