#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <vector>


class RemoveWall : public rclcpp::Node {
    public:
        RemoveWall(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        RemoveWall(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        void CallBack(const sensor_msgs::PointCloud2ConstPtr &msg);
        pcl::PointCloud<pcl::PointXYZI> DifferentialPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_base, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_test, float resolution);
        
    private:
        // Subscriber & Publisher
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc;

        // Parameter
        std::string compare_pcd_file, input_topic, output_topic;
        float resolution;

        // TF
};