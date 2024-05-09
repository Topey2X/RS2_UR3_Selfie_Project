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

nav_msgs::Path optimizePathWithLiftoff(const std::vector<std::vector<Point>>& lines, bool printDistances) {
    ROS_INFO("Optimizing path");
    
    std::vector<double> startPointDistances;
    std::vector<nav_msgs::Path> optimizedPaths;
    
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
        
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();

        for (const auto& segment : optimizedPath) {
            for (const auto& point : segment) {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = point.x;
                pose.pose.position.y = point.y;
                pose.pose.position.z = point.z;
                pose.pose.orientation.w = 1.0;
                path_msg.poses.push_back(pose);
            }
        }
        
        optimizedPaths.push_back(path_msg);
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

void writeToCsv(const nav_msgs::Path& optimizedPath, const std::string& filename) {
    std::ofstream csvFile(filename);
    if (!csvFile.is_open()) {
        std::cout << "Failed to open file: " << filename << std::endl;
        return;
    }

    csvFile << "x,y,z" << std::endl;
    for (size_t i = 0; i < optimizedPath.poses.size(); ++i) {
        const auto& pose = optimizedPath.poses[i];
        csvFile << pose.pose.position.x << "," << pose.pose.position.y << "," << pose.pose.position.z << std::endl;
        
        // Write an empty line after each segment
        if (i < optimizedPath.poses.size() - 1 && pose.pose.position.z != optimizedPath.poses[i + 1].pose.position.z) {
            csvFile << std::endl;
        }
    }

    csvFile.close();
    std::cout << filename << " generated." << std::endl;
}