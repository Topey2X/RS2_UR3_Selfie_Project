// mainWithRosMsgs.cpp
#include "function.h"
#include <ros/ros.h>
#include <ros/package.h>
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

        // Optimize the path with liftoff
        std::vector<std::vector<Point>> optimized_path = optimizePathWithLiftoff(lines, true);
        ROS_INFO("Optimized path size: %lu", optimized_path.size());




        // Write the optimized path to a CSV file
        std::string csv_filename = ros::package::getPath("robotSelfie") + "/src/optimized_path.csv";
        writeToCsv(optimized_path, csv_filename);

        sleep(10);


        // Convert the optimized path to nav_msgs::Path format
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();

        for (const auto& segment : optimized_path) {
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

        // Publish the optimized path
        path_pub.publish(path_msg);

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