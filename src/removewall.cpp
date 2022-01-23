#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <vector>
#include <ctime>
#include <fstream>
#include <chrono>

// tf

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// tf2
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "../include/TimeMeasurement.h"

class RemoveWall
{
public:
    RemoveWall();
    void CallBack(const sensor_msgs::PointCloud2ConstPtr &msg);
    pcl::PointCloud<pcl::PointXYZI> difference_extraction(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_base, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_test, float resolution);
    pcl::PointCloud<pcl::PointXYZI> difference_extraction2(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_base, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_test, float resolution);

private:
    // Node Handler
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate;

    // Publisher & Subscriber
    ros::Subscriber sub_pc;
    ros::Publisher pub_pc;

    // Params
    std::string compare_pcd_file, input_topic, output_topic;
    float resolution;

    // TF
    tf2_ros::Buffer tfBuffer;
    geometry_msgs::TransformStamped transform_stamped;

    // Compare PCD
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_base{new pcl::PointCloud<pcl::PointXYZI>};

    // Frame
    std::string map_frame, velodyne_frame;
};

RemoveWall::RemoveWall()
    : nhPrivate("~")
{
    // Init Params
    input_topic = "/points_no_extra";
    output_topic = "/points_no_wall";

    // GetParams
    nhPrivate.getParam("compare_pcd_file", compare_pcd_file);
    nhPrivate.getParam("resolution", resolution);
    nhPrivate.getParam("input_topic", input_topic);
    nhPrivate.getParam("output_topic", output_topic);

    // Publisher & Subscriber
    pub_pc = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 1);
    sub_pc = nh.subscribe(input_topic, 1, &RemoveWall::CallBack, this);

    // Frame
    map_frame = "map";
    velodyne_frame = "velodyne";

    // TF Listener
    tf2_ros::TransformListener listener(tfBuffer);

    // Map Load
    pcl::io::loadPCDFile(compare_pcd_file, *cloud_base);

    ros::spin();
}

void RemoveWall::CallBack(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    ROS_INFO_STREAM("CallBack");
    pcl::PointCloud<pcl::PointXYZI> cloud;
    ROS_INFO_STREAM(msg->header.frame_id);
    if (msg->header.frame_id == map_frame)
    {
        try
        {
            transform_stamped = tfBuffer.lookupTransform(map_frame, msg->header.frame_id, ros::Time(0));
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
        sensor_msgs::PointCloud2 transformed_cloud;
        Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
        pcl_ros::transformPointCloud(mat, *msg, transformed_cloud);
        pcl::fromROSMsg(transformed_cloud, cloud);
    }
    else
    {
        pcl::fromROSMsg(*msg, cloud);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(cloud));
    pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>);

    *result = RemoveWall::difference_extraction(cloud_base, cloud_ptr, resolution);

    sensor_msgs::PointCloud2::Ptr pc2_cloud(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*result, *pc2_cloud);
    pc2_cloud->header.frame_id = map_frame;
    pc2_cloud->header.stamp = ros::Time::now();

    try
    {
        transform_stamped = tfBuffer.lookupTransform(velodyne_frame, map_frame, ros::Time(0));
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    sensor_msgs::PointCloud2::Ptr pc2_cloud_transformed(new sensor_msgs::PointCloud2);
    // pcl_ros::transformPointCloud(map_frame, transform, *recent_cloud, transformed_cloud);
    Eigen::Matrix4f mat2 = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
    pcl_ros::transformPointCloud(mat2, *pc2_cloud, *pc2_cloud_transformed);

    pc2_cloud_transformed->header.frame_id = velodyne_frame;
    pc2_cloud_transformed->header.stamp = ros::Time::now();

    pub_pc.publish(pc2_cloud_transformed);
}

pcl::PointCloud<pcl::PointXYZI> RemoveWall::difference_extraction(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_base, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_test, float resolution = 0.00001f)
{
    //cloud_baseは元となる点群
    //cloud_testは比較対象の点群
    //cloud_diffは比較の結果差分とされた点群

    ROS_INFO_STREAM("difference_extraction points");

    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZI> octree(resolution); //Octreeを作成

    octree.setInputCloud(cloud_base); //元となる点群を入力
    octree.addPointsFromInputCloud();

    octree.switchBuffers(); //バッファの切り替え

    octree.setInputCloud(cloud_test); //比較対象の点群を入力
    octree.addPointsFromInputCloud();

    std::vector<int> newPointIdxVector; //

    octree.getPointIndicesFromNewVoxels(newPointIdxVector); //比較の結果差分と判断された点郡の情報を保管

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_diff(new pcl::PointCloud<pcl::PointXYZI>); //出力先

    //保管先のサイズの設定
    cloud_diff->width = cloud_base->points.size() + cloud_test->points.size();
    cloud_diff->height = 1;
    cloud_diff->points.resize(cloud_diff->width * cloud_diff->height);

    int n = 0; //差分点群の数を保存する
    for (size_t i = 0; i < newPointIdxVector.size(); i++)
    {
        cloud_diff->points[i].x = cloud_test->points[newPointIdxVector[i]].x;
        cloud_diff->points[i].y = cloud_test->points[newPointIdxVector[i]].y;
        cloud_diff->points[i].z = cloud_test->points[newPointIdxVector[i]].z;
        n++;
    }

    //差分点群のサイズの再設定
    cloud_diff->width = n;
    cloud_diff->height = 1;
    cloud_diff->points.resize(cloud_diff->width * cloud_diff->height);

    return *cloud_diff;
}

pcl::PointCloud<pcl::PointXYZI> RemoveWall::difference_extraction2(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_base, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_test, float resolution = 0.1f)
{
    srand((unsigned int)time(NULL));


    // Instantiate octree-based point cloud change detection class
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZI> octree(resolution);


    for (std::size_t i = 0; i < cloud_base->size(); ++i)
    {
        (*cloud_base)[i].x = 64.0f * rand() / (RAND_MAX + 1.0f);
        (*cloud_base)[i].y = 64.0f * rand() / (RAND_MAX + 1.0f);
        (*cloud_base)[i].z = 64.0f * rand() / (RAND_MAX + 1.0f);
    }

    // Add points from cloud_base to octree
    octree.setInputCloud(cloud_base);
    octree.addPointsFromInputCloud();

    // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
    octree.switchBuffers();


    for (std::size_t i = 0; i < cloud_test->size(); ++i)
    {
        (*cloud_test)[i].x = 64.0f * rand() / (RAND_MAX + 1.0f);
        (*cloud_test)[i].y = 64.0f * rand() / (RAND_MAX + 1.0f);
        (*cloud_test)[i].z = 64.0f * rand() / (RAND_MAX + 1.0f);
    }

    // Add points from cloud_test to octree
    octree.setInputCloud(cloud_test);
    octree.addPointsFromInputCloud();

    std::vector<int> newPointIdxVector;

    // Get vector of point indices from octree voxels which did not exist in previous buffer
    octree.getPointIndicesFromNewVoxels(newPointIdxVector);

    // Output points
    std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
    for (std::size_t i = 0; i < newPointIdxVector.size(); ++i)
        std::cout << i << "# Index:" << newPointIdxVector[i]
                  << "  Point:" << (*cloud_test)[newPointIdxVector[i]].x << " "
                  << (*cloud_test)[newPointIdxVector[i]].y << " "
                  << (*cloud_test)[newPointIdxVector[i]].z << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_diff(new pcl::PointCloud<pcl::PointXYZI>); //出力先

    //保管先のサイズの設定
    cloud_diff->width = cloud_base->points.size() + cloud_test->points.size();
    cloud_diff->height = 1;
    cloud_diff->points.resize(cloud_diff->width * cloud_diff->height);
    int n = 0; //差分点群の数を保存する

    for (size_t i = 0; i < newPointIdxVector.size(); i++)
    {
        cloud_diff->points[i].x = cloud_test->points[newPointIdxVector[i]].x;
        cloud_diff->points[i].y = cloud_test->points[newPointIdxVector[i]].y;
        cloud_diff->points[i].z = cloud_test->points[newPointIdxVector[i]].z;
        n++;
    }

    //差分点群のサイズの再設定
    cloud_diff->width = n;
    cloud_diff->height = 1;
    cloud_diff->points.resize(cloud_diff->width * cloud_diff->height);
    return *cloud_diff;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "removewall_node");
    RemoveWall removewall;
    return 0;
}