#include "function.h"

double calculateDistance(const Point& p1, const Point& p2) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}

int findNearestLine(const Point& currentPoint, const std::vector<std::vector<Point>>& lines, const std::vector<bool>& visitedLines) {
    int nearestLineIndex = -1;
    double minDistance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < lines.size(); ++i) {
        if (!visitedLines[i]) {
            const Point& startPoint = lines[i].front();
            const Point& endPoint = lines[i].back();

            double distanceToStart = calculateDistance(currentPoint, startPoint);
            double distanceToEnd = calculateDistance(currentPoint, endPoint);

            if (distanceToStart < minDistance) {
                minDistance = distanceToStart;
                nearestLineIndex = i;
            }

            if (distanceToEnd < minDistance) {
                minDistance = distanceToEnd;
                nearestLineIndex = i;
            }
        }
    }

    return nearestLineIndex;
}

double calculateTotalDistance(const std::vector<std::vector<Point>>& path) {
    double totalDistance = 0.0;
    for (const auto& segment : path) {
        for (size_t i = 1; i < segment.size(); ++i) {
            totalDistance += calculateDistance(segment[i - 1], segment[i]);
        }
    }
    return totalDistance;
}

std::vector<std::vector<Point>> optimizePathWithLiftoff(const std::vector<std::vector<Point>>& lines, bool printDistances) {
    ROS_INFO("Optimizing path");
    
    std::vector<double> startPointDistances;
    std::vector<std::vector<std::vector<Point>>> optimizedPaths;
    
    // Calculate the optimized path for each possible starting point
    for (const auto& line : lines) {
        std::vector<std::vector<Point>> tempLines = lines;
        std::vector<std::vector<Point>> optimizedPath;
        std::vector<bool> visitedLines(tempLines.size(), false);
        
        Point currentPoint = line.front();
        currentPoint.z = 0;
        
        while (true) {
            int nearestLineIndex = findNearestLine(currentPoint, tempLines, visitedLines);

            if (nearestLineIndex == -1) {
                break;
            }

            visitedLines[nearestLineIndex] = true;
            const std::vector<Point>& nearestLine = tempLines[nearestLineIndex];

            Point transitionPoint;
            if (calculateDistance(currentPoint, nearestLine.front()) < calculateDistance(currentPoint, nearestLine.back())) {
                transitionPoint = nearestLine.front();
            } else {
                transitionPoint = nearestLine.back();
            }

            double transitionDistance = calculateDistance(currentPoint, transitionPoint);
            std::vector<Point> transitionSegment;
            transitionSegment.push_back({ currentPoint.x, currentPoint.y, 0.0001 });
            transitionSegment.push_back({ transitionPoint.x, transitionPoint.y, 0.0001 });
            optimizedPath.push_back(transitionSegment);

            std::vector<Point> segment;
            if (calculateDistance(currentPoint, nearestLine.front()) < calculateDistance(currentPoint, nearestLine.back())) {
                segment = nearestLine;
            } else {
                segment = std::vector<Point>(nearestLine.rbegin(), nearestLine.rend());
            }
            for (auto& point : segment) {
                point.z = 0;
            }
            optimizedPath.push_back(segment);

            std::vector<Point> liftoffSegment;
            liftoffSegment.push_back({ segment.back().x, segment.back().y, 0 });
            liftoffSegment.push_back({ segment.back().x, segment.back().y, 0.0001 });
            optimizedPath.push_back(liftoffSegment);

            currentPoint = segment.back();
        }
        
        double totalDistance = calculateTotalDistance(optimizedPath);
        startPointDistances.push_back(totalDistance);
        
        optimizedPaths.push_back(optimizedPath);
    }
    
    // Find the index of the starting point with the minimum total distance
    size_t minDistanceIndex = std::min_element(startPointDistances.begin(), startPointDistances.end()) - startPointDistances.begin();
    
    if (printDistances) {
        ROS_INFO("Distances for different starting points:");
        for (size_t i = 0; i < startPointDistances.size(); ++i) {
            ROS_INFO("Starting point %lu: %f", i, startPointDistances[i]);
        }
    }
    
    ROS_INFO("Best starting point: %lu", minDistanceIndex);
    ROS_INFO("Minimum total distance: %f", startPointDistances[minDistanceIndex]);
    
    return optimizedPaths[minDistanceIndex];
}
void writeToCsv(const std::vector<std::vector<Point>>& optimizedPath, const std::string& filename) {
    std::ofstream csvFile(filename);
    if (!csvFile.is_open()) {
        ROS_ERROR("Failed to open file: %s", filename.c_str());
        return;
    }

    csvFile << "x,y,z" << std::endl;
    for (const auto& segment : optimizedPath) {
        for (const auto& point : segment) {
            csvFile << point.x << "," << point.y << "," << point.z << std::endl;
        }
        csvFile << std::endl;
    }

    csvFile.close();
    ROS_INFO("CSV file generated: %s", filename.c_str());
}