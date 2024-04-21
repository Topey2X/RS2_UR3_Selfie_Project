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

int main() {
    std::vector<std::vector<Point>> lines = {
        // ... (existing lines data with x and y coordinates)
    };

    std::vector<Point> optimizedPath;
    std::vector<bool> visitedLines(lines.size(), false);

    Point currentPoint = lines[0].front();
    currentPoint.z = 0;
    optimizedPath.push_back(currentPoint);

    while (true) {
        int nearestLineIndex = findNearestLine(currentPoint, lines, visitedLines);

        if (nearestLineIndex == -1) {
            break;
        }

        visitedLines[nearestLineIndex] = true;
        const std::vector<Point>& nearestLine = lines[nearestLineIndex];

        Point transitionPoint;
        if (calculateDistance(currentPoint, nearestLine.front()) < calculateDistance(currentPoint, nearestLine.back())) {
            transitionPoint = nearestLine.front();
        } else {
            transitionPoint = nearestLine.back();
        }

        double transitionDistance = calculateDistance(currentPoint, transitionPoint);
        if (transitionDistance >= 10) {
            currentPoint.z = 1;
            optimizedPath.push_back(currentPoint);
            transitionPoint.z = 1;
            optimizedPath.push_back(transitionPoint);
            transitionPoint.z = 0;
            optimizedPath.push_back(transitionPoint);
        }

        if (calculateDistance(currentPoint, nearestLine.front()) < calculateDistance(currentPoint, nearestLine.back())) {
            for (const auto& point : nearestLine) {
                optimizedPath.push_back({ point.x, point.y, 0 });
            }
            currentPoint = nearestLine.back();
        } else {
            for (auto it = nearestLine.rbegin(); it != nearestLine.rend(); ++it) {
                optimizedPath.push_back({ it->x, it->y, 0 });
            }
            currentPoint = nearestLine.front();
        }
    }

    // Output the optimized path with z-axis to a CSV file
    std::ofstream csvFile("optimized_path_with_z.csv");
    csvFile << "x,y,z" << std::endl;
    for (const auto& point : optimizedPath) {
        csvFile << point.x << "," << point.y << "," << point.z << std::endl;
    }
    csvFile.close();

    std::cout << "Optimized path with z-axis has been saved to optimized_path_with_z.csv" << std::endl;

    return 0;
}