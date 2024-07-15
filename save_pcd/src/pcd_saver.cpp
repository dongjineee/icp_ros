#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sys/stat.h>
#include <sstream>
#include <fstream>

int frame_count = 0;  // 전역 변수로 순서 카운터를 선언합니다.
std::ofstream odom_file;  // odom 데이터를 기록할 파일 스트림

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const nav_msgs::Odometry::ConstPtr& odom_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    std::string directory = "/home/dongjineee/24_project/24_1_project/24_1_localization/src/icp_ws/src/save_pcd/save_file/";
    std::stringstream ss;
    ss << directory << "lidar_" << frame_count << ".pcd";
    std::string filename = ss.str();

    struct stat info;
    if (stat(directory.c_str(), &info) != 0) {
        ROS_INFO("Directory %s does not exist. Creating now...", directory.c_str());
        if (mkdir(directory.c_str(), 0777) == -1) {
            ROS_ERROR("Error creating directory %s", directory.c_str());
            return;
        } else {
            ROS_INFO("Successfully created directory %s", directory.c_str());
        }
    }

    // PCD 파일로 저장
    if (pcl::io::savePCDFileASCII(filename, *cloud) == 0) {
        ROS_INFO("Successfully saved %ld data points to %s", cloud->points.size(), filename.c_str());
    } else {
        ROS_ERROR("Failed to save PCD file to %s", filename.c_str());
    }

    if (odom_file.is_open()) {
        odom_file << odom_msg->pose.pose.position.x << " "
                  << odom_msg->pose.pose.position.y << " "
                  << odom_msg->pose.pose.position.z << " "
                  << odom_msg->pose.pose.orientation.x << " "
                  << odom_msg->pose.pose.orientation.y << " "
                  << odom_msg->pose.pose.orientation.z << " "
                  << odom_msg->pose.pose.orientation.w << "\n";
        ROS_INFO("Successfully saved odom data for frame %d", frame_count);
    } else {
        ROS_ERROR("Failed to save odom data for frame %d", frame_count);
    }

    frame_count++;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcd_saver");
    ros::NodeHandle nh;

    std::string odom_directory = "/home/dongjineee/24_project/24_1_project/24_1_localization/src/icp_ws/src/save_pcd/save_file/odom/";
    std::string odom_filename = odom_directory + "odom_data.txt";

    // odom 데이터를 저장할 디렉토리 생성
    struct stat info;
    if (stat(odom_directory.c_str(), &info) != 0) {
        ROS_INFO("Directory %s does not exist. Creating now...", odom_directory.c_str());
        if (mkdir(odom_directory.c_str(), 0777) == -1) {
            ROS_ERROR("Error creating directory %s", odom_directory.c_str());
            return -1;
        } else {
            ROS_INFO("Successfully created directory %s", odom_directory.c_str());
        }
    }

    // odom 데이터를 기록할 파일 열기
    odom_file.open(odom_filename);
    if (!odom_file.is_open()) {
        ROS_ERROR("Failed to open odom data file at %s", odom_filename.c_str());
        return -1;
    }

    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, "/platform/velodyne_points", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/platform/odometry", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pcl_sub, odom_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ROS_INFO("PCD Saver Node Started");
    ros::spin();

    // 노드 종료 시 파일 닫기
    odom_file.close();
    return 0;
}
