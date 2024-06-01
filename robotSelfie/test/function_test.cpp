#include "../src/function.h"
#include <gtest/gtest.h>

// Test case for calculateDistance function
TEST(FunctionTest, CalculateDistance) {
    Point p1{0, 0, 0};
    Point p2{3, 4, 0};
    double expected = 5.0;
    double result = calculateDistance(p1, p2);
    std::cout << "CalculateDistance Result: " << result << std::endl;
    EXPECT_DOUBLE_EQ(expected, result);
}

// Test case for findNearestLine function
TEST(FunctionTest, FindNearestLine) {
    Point currentPoint{0, 0, 0};
    std::vector<std::vector<Point>> lines{
        {{1, 1, 0}, {2, 2, 0}},
        {{3, 3, 0}, {4, 4, 0}},
        {{5, 5, 0}, {6, 6, 0}}
    };
    std::vector<bool> visitedLines{false, false, false};
    int expected = 0;
    int result = findNearestLine(currentPoint, lines, visitedLines);
    std::cout << "FindNearestLine Result: " << result << std::endl;
    EXPECT_EQ(expected, result);
}

// Test case for calculateTotalDistance function
TEST(FunctionTest, CalculateTotalDistance) {
    std::vector<std::vector<Point>> path{
        {{0, 0, 0}, {1, 1, 0}},
        {{1, 1, 0}, {2, 2, 0}},
        {{2, 2, 0}, {3, 3, 0}}
    };
    double expected = 4.24264068711928;
    double result = calculateTotalDistance(path);
    std::cout << "CalculateTotalDistance Result: " << result << std::endl;
    EXPECT_NEAR(expected, result, 1e-8);
}

// Test case for optimizePathWithLiftoff function
TEST(FunctionTest, OptimizePathWithLiftoff) {
    std::vector<std::vector<Point>> lines{
        {{0, 0, 0}, {1, 1, 0}},
        {{2, 2, 0}, {3, 3, 0}},
        {{4, 4, 0}, {5, 5, 0}}
    };
    std::vector<std::vector<Point>> expected{
        {{0, 0, 1}, {0, 0, 1}},
        {{0, 0, 0}, {1, 1, 0}},
        {{1, 1, 1}, {2, 2, 1}},
        {{2, 2, 0}, {3, 3, 0}},
        {{3, 3, 1}, {4, 4, 1}},
        {{4, 4, 0}, {5, 5, 0}}
    };
    std::vector<std::vector<Point>> result = optimizePathWithLiftoff(lines);

    // Print the expected vector
    std::cout << "Expected:" << std::endl;
    for (const auto& segment : expected) {
        for (const auto& point : segment) {
            std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ") ";
        }
        std::cout << std::endl;
    }

    // Print the result vector
    std::cout << "Result:" << std::endl;
    for (const auto& segment : result) {
        for (const auto& point : segment) {
            std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ") ";
        }
        std::cout << std::endl;
    }

    EXPECT_EQ(expected, result);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}