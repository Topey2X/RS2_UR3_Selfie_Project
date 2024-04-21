
#include "nearestLineEndpoint.h"

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

std::vector<std::vector<Point>> optimizePathWithLiftoff(const std::vector<std::vector<Point>>& lines) {
    std::vector<std::vector<Point>> optimizedPath;
    std::vector<bool> visitedLines(lines.size(), false);
    Point currentPoint = lines[0].front();
    currentPoint.z = 0;

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
        std::vector<Point> transitionSegment;
        transitionSegment.push_back({ currentPoint.x, currentPoint.y, 1 });
        transitionSegment.push_back({ transitionPoint.x, transitionPoint.y, 1 });
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
        liftoffSegment.push_back({ segment.back().x, segment.back().y, 1 });
        optimizedPath.push_back(liftoffSegment);

        currentPoint = segment.back();
    }

    double totalDistance = calculateTotalDistance(optimizedPath);
    std::cout << "Total distance traveled: " << totalDistance << std::endl;

    return optimizedPath;
}