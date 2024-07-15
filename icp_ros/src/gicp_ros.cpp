#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <string>

using namespace std;

bool readOdomData(const std::string& filename, std::vector<Eigen::Matrix4f>& transforms) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open odom file: " << filename << std::endl;
        return false;
    }

    std::string line;
    std::vector<double> data;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        double value;
        while (iss >> value) {
            data.push_back(value);
        }
    }

    if (data.size() != 14) {
        std::cerr << "Incorrect odom data format: expected 14 values, got " << data.size() << std::endl;
        return false;
    }

    for (int i = 0; i < 2; ++i) {
        Eigen::Quaternionf q(data[6 + i * 7], data[3 + i * 7], data[4 + i * 7], data[5 + i * 7]);
        Eigen::Matrix3f rotation = q.toRotationMatrix();

        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = rotation;
        transform(0, 3) = data[0 + i * 7];
        transform(1, 3) = data[1 + i * 7];
        transform(2, 3) = data[2 + i * 7];

        transforms.push_back(transform);
    }

    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gicp_registration_node");
    ros::NodeHandle nh;

    ros::Publisher pub_source = nh.advertise<sensor_msgs::PointCloud2>("source_cloud", 1);
    ros::Publisher pub_target = nh.advertise<sensor_msgs::PointCloud2>("target_cloud", 1);

    ros::Publisher pub_initial_source = nh.advertise<sensor_msgs::PointCloud2>("initial_source_cloud", 1);
    ros::Publisher pub_initial_target = nh.advertise<sensor_msgs::PointCloud2>("initial_target_cloud", 1);

    ros::Publisher pub_aligned = nh.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 1);
    ros::Publisher pub_transformed_target = nh.advertise<sensor_msgs::PointCloud2>("transformed_target_cloud", 1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>());

    // PCD 파일 경로
    std::string source_file = "/home/dongjineee/24_project/24_1_project/24_1_localization/src/icp_ws/src/save_pcd/save_file/lidar_0.pcd";
    std::string target_file = "/home/dongjineee/24_project/24_1_project/24_1_localization/src/icp_ws/src/save_pcd/save_file/lidar_1.pcd";
    std::string odom_file = "/home/dongjineee/24_project/24_1_project/24_1_localization/src/icp_ws/src/save_pcd/save_file/odom/odom_data.txt";

    // PCD 파일 로드
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(source_file, *cloud_source) == -1) {
        PCL_ERROR("Couldn't read source file %s \n", source_file.c_str());
        return (-1);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(target_file, *cloud_target) == -1) {
        PCL_ERROR("Couldn't read target file %s \n", target_file.c_str());
        return (-1);
    }

    // odom 데이터 읽기
    std::vector<Eigen::Matrix4f> transforms;
    if (!readOdomData(odom_file, transforms)) {
        return -1;
    }

    Eigen::Matrix4f initial_guess_source = transforms[0];
    Eigen::Matrix4f initial_guess_target = transforms[1];

    ros::Duration(1.0).sleep();
    ros::spinOnce();

    sensor_msgs::PointCloud2 source_msg;
    pcl::toROSMsg(*cloud_source, source_msg);
    source_msg.header.frame_id = "map";
    source_msg.header.stamp = ros::Time::now();
    pub_source.publish(source_msg);

    ros::Duration(1.0).sleep();
    ros::spinOnce();

    sensor_msgs::PointCloud2 target_msg;
    pcl::toROSMsg(*cloud_target, target_msg);
    target_msg.header.frame_id = "map";
    target_msg.header.stamp = ros::Time::now();
    pub_target.publish(target_msg);

    ros::Duration(1.0).sleep();
    ros::spinOnce();

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud_source, *transformed_source, initial_guess_source);

    sensor_msgs::PointCloud2 transformed_source_msg;
    pcl::toROSMsg(*transformed_source, transformed_source_msg);
    transformed_source_msg.header.frame_id = "map";
    transformed_source_msg.header.stamp = ros::Time::now();
    pub_initial_source.publish(transformed_source_msg);

    ros::Duration(1.0).sleep();
    ros::spinOnce();

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_target(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud_target, *transformed_target, initial_guess_target);

    sensor_msgs::PointCloud2 transformed_target_msg;
    pcl::toROSMsg(*transformed_target, transformed_target_msg);
    transformed_target_msg.header.frame_id = "map";
    transformed_target_msg.header.stamp = ros::Time::now();
    pub_initial_target.publish(transformed_target_msg);

    ros::Duration(1.0).sleep();
    ros::spinOnce();

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(transformed_source);
    gicp.setInputTarget(transformed_target);
    gicp.setMaximumIterations(10);
    gicp.setMaxCorrespondenceDistance(0.01);

    pcl::PointCloud<pcl::PointXYZ> Final;
    gicp.align(Final);

    if (gicp.hasConverged()) {
        std::cout << "GICP has converged." << std::endl;
        std::cout << "Score: " << gicp.getFitnessScore() << std::endl;
        std::cout << "Transformation Matrix:" << std::endl;
        std::cout << gicp.getFinalTransformation() << std::endl;

        sensor_msgs::PointCloud2 aligned_msg;
        pcl::toROSMsg(Final, aligned_msg);
        aligned_msg.header.frame_id = "map";
        aligned_msg.header.stamp = ros::Time::now();
        pub_aligned.publish(aligned_msg);
        ROS_INFO("Published aligned cloud");

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_target_1(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*transformed_target, *transformed_target_1, gicp.getFinalTransformation());

        sensor_msgs::PointCloud2 transformed_target_final_msg;
        pcl::toROSMsg(*transformed_target_1, transformed_target_final_msg);
        transformed_target_final_msg.header.frame_id = "map";
        transformed_target_final_msg.header.stamp = ros::Time::now();
        pub_transformed_target.publish(transformed_target_final_msg);
        ROS_INFO("Published transformed target cloud");

    } else {
        std::cout << "GICP did not converge." << std::endl;
        return (-1);
    }

    ros::spinOnce();

    return 0;
}
