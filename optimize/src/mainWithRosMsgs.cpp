#include "function.h"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <optimize/Path.h>

void linesCallback(const optimize::Path::ConstPtr& msg) {
    std::vector<std::vector<Point>> lines;

    // Convert the received lines data to the required format
    for (const auto& line : msg->lines) {
        std::vector<Point> points;
        for (const auto& point : line.points) {
            points.push_back({point.x, point.y, point.z});
        }
        lines.push_back(points);
    }

    // Convert the lines from pixels to millimeters
    double pixelsPerMM = 1.0; // Adjust this value based on your image resolution
    double offsetX = 0.0;
    double offsetY = 0.0;
    double offsetZ = 0.0;
    std::vector<std::vector<Point>> convertedLines = convertToMM(lines, pixelsPerMM, offsetX, offsetY);

    // Optimize the path with liftoff
    nav_msgs::Path optimized_path = optimizePathWithLiftoff(convertedLines);

    // Write the optimized path to a CSV file
    // writeToCsv(optimized_path, "optimized_path.csv");

    // Publish the optimized path
    static ros::Publisher path_pub = ros::NodeHandle().advertise<nav_msgs::Path>("optimized_path", 1);
    path_pub.publish(optimized_path);
}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "path_optimizer");
    ros::NodeHandle nh;

    // Create a subscriber for the lines data
    ros::Subscriber lines_sub = nh.subscribe("lines_topic", 1, linesCallback);

    // Spin and process callbacks
    ros::spin();

    return 0;
}