#include "function.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Path.h>
#include "robotSelfie/ContourList.h"
#include "robotSelfie/Contour.h"

ros::Publisher path_pub;
nav_msgs::Path optimized_path_msg;
bool optimizedPathReceived = false;

// Publish the optimized path at a fixed rate
void publishOptimizedPath(const ros::TimerEvent&) {
    if (optimizedPathReceived) {
        path_pub.publish(optimized_path_msg);
    }
}

// Callback function to process the received contours data
void contoursCallback(const robotSelfie::ContourList::ConstPtr& msg) {
    if (!optimizedPathReceived) {
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

        // Write the optimized path to a TXT file
        std::string txt_filename = ros::package::getPath("robotSelfie") + "/src/optimized_path.txt";
        writeToTxt(optimized_path, txt_filename);

        // Convert the optimized path to nav_msgs::Path format
        optimized_path_msg.header.frame_id = "map";
        optimized_path_msg.header.stamp = ros::Time::now();

        for (const auto& segment : optimized_path) {
            for (const auto& point : segment) {
                //if (segment.size() > 2) {
                    geometry_msgs::PoseStamped pose;
                    pose.header.frame_id = "map";
                    pose.header.stamp = ros::Time::now();
                    pose.pose.position.x = point.x;
                    pose.pose.position.y = point.y;
                    pose.pose.position.z = point.z;
                    pose.pose.orientation.w = 1.0;
                    optimized_path_msg.poses.push_back(pose);
                //}
            }
        }

        optimizedPathReceived = true;
    }
}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "path_optimizer");
    ros::NodeHandle nh;

    // Create a subscriber for the contours data
    ros::Subscriber contours_sub = nh.subscribe("contours", 1, contoursCallback);
    path_pub = nh.advertise<nav_msgs::Path>("optimized_path", 1);

    // Create a timer to publish the optimized path at a fixed rate (e.g., 0.2 Hz)
    ros::Timer timer = nh.createTimer(ros::Duration(5), publishOptimizedPath);

    // Spin and process callbacks
    ros::spin();

    return 0;
}