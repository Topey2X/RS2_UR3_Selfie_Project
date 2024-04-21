#include <gtest/gtest.h>
#include "nearestLineEndpoint.h"

// Unit tests will be added here
TEST(NearestLineEndpointTest, CalculateDistance) {
    Point p1 = {0, 0, 0};
    Point p2 = {3, 4, 0};
    EXPECT_DOUBLE_EQ(calculateDistance(p1, p2), 5.0);

    Point p3 = {1, 1, 0};
    Point p4 = {4, 5, 0};
    EXPECT_DOUBLE_EQ(calculateDistance(p3, p4), 5.0);
}

TEST(NearestLineEndpointTest, FindNearestLine) {
    std::vector<std::vector<Point>> lines = {
        {{0, 0, 0}, {1, 1, 0}},
        {{2, 2, 0}, {3, 3, 0}},
        {{4, 4, 0}, {5, 5, 0}}
    };
    std::vector<bool> visitedLines(lines.size(), false);

    Point currentPoint = {0, 0, 0};
    EXPECT_EQ(findNearestLine(currentPoint, lines, visitedLines), 0);

    currentPoint = {2.49, 2.49, 0};
    EXPECT_EQ(findNearestLine(currentPoint, lines, visitedLines), 1);

    visitedLines[1] = true;
    EXPECT_EQ(findNearestLine(currentPoint, lines, visitedLines), 2);
}

TEST(NearestLineEndpointTest, CalculateTotalDistance) {
    std::vector<std::vector<Point>> path = {
        {{0, 0, 0}, {1, 1, 0}},
        {{1, 1, 0}, {2, 2, 0}},
        {{2, 2, 0}, {3, 3, 0}}
    };
    EXPECT_DOUBLE_EQ(calculateTotalDistance(path), 3.0 * std::sqrt(2));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}