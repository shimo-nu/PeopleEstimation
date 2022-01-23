#pragma once


#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>


template<typename PointT>
geometry_msgs::Point Centorids(pcl::PointCloud<PointT> cloud)
{
    geometry_msgs::Point _centroids;
    for (size_t i = 0; i < cloud.points.size();i++)
    {
        _centroids.x += cloud.points[i].x / cloud.points.size();
        _centroids.y += cloud.points[i].y / cloud.points.size();
        _centroids.z += cloud.points[i].z / cloud.points.size();
    }
    return _centroids;
}

template<typename PointT>
geometry_msgs::Point maxPoint(pcl::PointCloud<PointT> cloud)
{
    geometry_msgs::Point _max_point;
    _max_point.x = -1e7;
    _max_point.y = -1e7;
    _max_point.z = -1e7;
    for (size_t i = 0; i < cloud.points.size();i++)
    {
        if (_max_point.x < cloud.points[i].x) _max_point.x = cloud.point[i].x;
        if (_max_point.y < cloud.points[i].y) _max_point.y = cloud.point[i].y;
        if (_max_point.z < cloud.points[i].z) _max_point.z = cloud.point[i].z;
    }
    return _max_point;
}


template<typename PointT>
geometry_msgs::Point minPoint(pcl::PointCloud<PointT> cloud)
{
    geometry_msgs::Point _min_point;
    _min_point.x = -1e7;
    _min_point.y = -1e7;
    _min_point.z = -1e7;
    for (size_t i = 0; i < cloud.points.size();i++)
    {
        if (_min_point.x > cloud.points[i].x) _min_point.x = cloud.point[i].x;
        if (_min_point.y > cloud.points[i].y) _min_point.y = cloud.point[i].y;
        if (_min_point.z > cloud.points[i].z) _min_point.z = cloud.point[i].z;
    }
    return _min_point;
}


tf::Transform ImuToTransform(const sensor_msgs::Imu& msg)
{
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    double g = 9.8;
    double r,p, y;
    // tf::Quaternion q(
        // msg.orientation.x,
        // msg.orientation.y,
        // msg.orientation.z,
        // msg.orientation.w
    // );
    // tf::Quaternion q;
    // tf::Matrix3x3(q).getRPY(r, p, y);
    // r = atan(msg.linear_acceleration.y/msg.linear_acceleration.z);
    // p = atan(-msg.linear_acceleration.x/(sqrt(pow(msg.linear_acceleration.y,2)+pow(msg.linear_acceleration.z,2))));
    // q.setRPY(r, q, y);
    // y = 2 * atan(sqrt(msg.orientation.x * msg.orientation.x + msg.orientation.y * msg.orientation.y + msg.orientation.z * msg.orientation.z) / msg.orientation.w);
    // q.setRPY(0, 0, y);
    tf::Quaternion quat;
    quaternionMsgToTF(msg.orientation, quat);
    tf::Matrix3x3(quat).getRPY(r, p, y); 
    transform.setRotation(quat);
    return transform;
}