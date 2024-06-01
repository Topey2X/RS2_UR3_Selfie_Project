#ifndef FUNCTIONROS_H
#define FUNCTIONROS_H

#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <fstream>
#include <ros/ros.h>
#include <geometry_msgs/Point32.h>

// Struct to represent a 3D point
struct Point {
    double x;
    double y;
    double z;


    //Can't explain what is happening here but it is a fix from AI Enquiries for me to be able to RUN Tests.
    bool operator==(const Point& other) const {
    return x == other.x && y == other.y && z == other.z;
    }
};



// Function declarations
double calculateDistance(const Point& p1, const Point& p2);
int findNearestLine(const Point& currentPoint, const std::vector<std::vector<Point>>& lines, const std::vector<bool>& visitedLines);
double calculateTotalDistance(const std::vector<std::vector<Point>>& path);
std::vector<std::vector<Point>> optimizePathWithLiftoff(const std::vector<std::vector<Point>>& lines, bool printDistances = false);
void writeToCsv(const std::vector<std::vector<Point>>& optimizedPath, const std::string& filename);
void writeToTxt(const std::vector<std::vector<Point>>& optimizedPath, const std::string& filename);

#endif