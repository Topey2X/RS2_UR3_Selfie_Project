#include "function.h"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include "robotSelfie/ContourList.h"
#include "robotSelfie/Contour.h"

ros::Publisher path_pub;

bool messageReceived = false;

void contoursCallback(const robotSelfie::ContourList::ConstPtr& msg) {
    if (!messageReceived) {
        ROS_INFO("Received contours message");
        ROS_INFO("Number of contours: %lu", msg->contours.size());
        std::vector<std::vector<Point>> lines;

        // Convert the received contours data to the required format
        for (const auto& contour : msg->contours) {
            std::vector<Point> points;
            for (const auto& point : contour.points) {
                points.push_back({point.x, point.y, point.z});
            }
            lines.push_back(points);
        }

        // Convert the lines from pixels to millimeters
        // double pixelsPerMM = 1.0; // Adjust this value based on your image resolution
        // double offsetX = 0.0;
        // double offsetY = 0.0;
        // double offsetZ = 0.0;
        // std::vector<std::vector<Point>> convertedLines = convertToMM(lines, pixelsPerMM, offsetX, offsetY);
        
        // Optimize the path with liftoff
        nav_msgs::Path optimized_path = optimizePathWithLiftoff(lines, true);
        ROS_INFO("Optimized path size: %lu", optimized_path.poses.size());

        // Publish the optimized path
        path_pub.publish(optimized_path);

        messageReceived = true;
    }
}



int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "path_optimizer");
    ros::NodeHandle nh;

    // Create a subscriber for the contours data
    ros::Subscriber contours_sub = nh.subscribe("contours", 1, contoursCallback);
    path_pub = nh.advertise<nav_msgs::Path>("optimized_path", 1);

    // Spin and process callbacks
    ros::spin();

    return 0;
}