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

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "gicp_registration_node");
    ros::NodeHandle nh;

    ros::Publisher pub_source = nh.advertise<sensor_msgs::PointCloud2>("source_cloud", 1);
    ros::Publisher pub_initial = nh.advertise<sensor_msgs::PointCloud2>("initial_cloud", 1);
    ros::Publisher pub_target = nh.advertise<sensor_msgs::PointCloud2>("target_cloud", 1);
    ros::Publisher pub_aligned = nh.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 1);
    ros::Publisher pub_transformed_target = nh.advertise<sensor_msgs::PointCloud2>("transformed_target_cloud", 1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>());

    // PCD 파일 경로
    std::string source_file = "/home/dongjineee/24_project/24_1_project/24_1_localization/src/icp_ws/src/save_pcd/save_file/lidar_1.pcd";
    std::string target_file = "/home/dongjineee/24_project/24_1_project/24_1_localization/src/icp_ws/src/save_pcd/save_file/lidar_5.pcd";

    // PCD 파일 로드
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(source_file, *cloud_source) == -1) {
        PCL_ERROR("Couldn't read source file %s \n", source_file.c_str());
        return (-1);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(target_file, *cloud_target) == -1) {
        PCL_ERROR("Couldn't read target file %s \n", target_file.c_str());
        return (-1);
    }

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

    //initialize_icp//
    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();

    float theta_x = 0; 
    float theta_y = 0;
    float theta_z = -(130.0 / 180.0) * M_PI;

    // x축 회전 행렬
    Eigen::Matrix4f rotation_x = Eigen::Matrix4f::Identity();
    rotation_x(1, 1) = cos(theta_x);
    rotation_x(1, 2) = -sin(theta_x);
    rotation_x(2, 1) = sin(theta_x);
    rotation_x(2, 2) = cos(theta_x);

    // y축 회전 행렬
    Eigen::Matrix4f rotation_y = Eigen::Matrix4f::Identity();
    rotation_y(0, 0) = cos(theta_y);
    rotation_y(0, 2) = sin(theta_y);
    rotation_y(2, 0) = -sin(theta_y);
    rotation_y(2, 2) = cos(theta_y);

    // z축 회전 행렬
    Eigen::Matrix4f rotation_z = Eigen::Matrix4f::Identity();
    rotation_z(0, 0) = cos(theta_z);
    rotation_z(0, 1) = -sin(theta_z);
    rotation_z(1, 0) = sin(theta_z);
    rotation_z(1, 1) = cos(theta_z);

    // 전체 회전 행렬
    Eigen::Matrix4f rotation = rotation_z * rotation_y * rotation_x;

    initial_guess.block<3, 3>(0, 0) = rotation.block<3, 3>(0, 0);
    initial_guess(0, 3) = 1.0; 
    initial_guess(1, 3) = 2.0; 
    initial_guess(2, 3) = 3.0; 

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_target(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud_target, *transformed_target, initial_guess);

    // 초기 맞춤 후 퍼블리시
    sensor_msgs::PointCloud2 transformed_target_msg;
    pcl::toROSMsg(*transformed_target, transformed_target_msg);
    transformed_target_msg.header.frame_id = "map";
    transformed_target_msg.header.stamp = ros::Time::now();
    pub_initial.publish(transformed_target_msg);

    ros::Duration(1.0).sleep();
    ros::spinOnce();

    // GICP 객체 생성 및 설정
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(cloud_source);
    gicp.setInputTarget(transformed_target);
    gicp.setMaximumIterations(10000);
    gicp.setMaxCorrespondenceDistance(0.1);

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
