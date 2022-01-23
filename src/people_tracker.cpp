#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


// PCL
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/io/pcd_io.h>


#include <iostream>
#include <vector>
#include <ctime>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <complex>

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


#include "jsk_recognition_msgs/BoundingBox.h"
#include "jsk_recognition_msgs/BoundingBoxArray.h"
#include <visualization_msgs/MarkerArray.h>

#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/DetectedObject.h>


class AutowareDetection 
{
    private:
        ros::NodeHandle nh;
        ros::NodeHandle nhPrivate;

        ros::Subscriber cluster_sub;

        ros::Publisher autoware_detection_pub;

    public:
        AutowareDetection();

        void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msgs);
};