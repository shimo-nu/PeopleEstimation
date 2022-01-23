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
#include "jsk_rviz_plugins/OverlayText.h"
#include "jsk_rviz_plugins/Pictogram.h"
#include "jsk_rviz_plugins/PictogramArray.h"
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int32.h>

#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/DetectedObject.h>



class PeopleCounter {
    private:
        ros::NodeHandle nh;
        ros::NodeHandle nhPrivate;
        
        ros::Subscriber pc_sub;
        ros::Subscriber bb_sub;
        ros::Subscriber classifier_sub;
        // ros::Subscriber cluster_sub;

        ros::Publisher bb_pub;
        ros::Publisher classifier_pub;
        ros::Publisher text_pub;
        ros::Publisher person_num_pub;
        ros::Publisher person_pict_pub;



        int person_num;
        std::vector<std::string> person;

        visualization_msgs::MarkerArray lineArray;
        visualization_msgs::MarkerArray before_people_state;

        jsk_recognition_msgs::BoundingBoxArray bb_array;

        std::string input_topic, marker_topic_name;

        float x_center, y_center, radius;

    public:
        PeopleCounter();

        // void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msgs);
        void BoundingBoxCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr &msgs);
        void MarkerArrayCallback(const visualization_msgs::MarkerArray::ConstPtr &msgs);
        // void ClusterCallback(const visualization_msgs::MarkerArray::ConstPtr &msgs);
        void LineCallback(const visualization_msgs::MarkerArray::ConstPtr &msgs);
        double calculateL2(geometry_msgs::Point a, geometry_msgs::Point b);
        double calculateL2(geometry_msgs::Pose a, geometry_msgs::Pose b);
        geometry_msgs::Point calculateCentroids(std::vector<geometry_msgs::Point> points);

};

PeopleCounter::PeopleCounter() : nhPrivate("~") {
    marker_topic_name = "/detection/object_tracker/objects_markers";

    nhPrivate.getParam("input_topic", input_topic);
    nhPrivate.getParam("marker_topic", marker_topic_name);
    nhPrivate.getParam("center_x", x_center);
    nhPrivate.getParam("center_y", y_center);
    nhPrivate.getParam("radius", radius);

    std::cout << marker_topic_name << std::endl;
    // pc_sub = nh.subscribe<sensor_msgs::PointCloud2>(input_topic, 20, &PeopleCounter::PointCloudCallback, this);
    bb_sub = nh.subscribe<jsk_recognition_msgs::BoundingBoxArray>("/cluster_box", 10, &PeopleCounter::BoundingBoxCallback, this);    
    classifier_sub = nh.subscribe<visualization_msgs::MarkerArray>(marker_topic_name, 10, &PeopleCounter::MarkerArrayCallback, this);
    // cluster_sub = nh.subscribe<visualization_msgs::MarkerArray>("/detection/object_tracker/object", 1, &People::Counter::ClusterCallback, this);
    
    bb_pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/people_track_box", 5);
    classifier_pub = nh.advertise<visualization_msgs::MarkerArray>("/people_counter/markers", 5);
    text_pub = nh.advertise<visualization_msgs::MarkerArray>("/people_counter/Id", 1);
    person_num_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/person_num", 1);
    person_pict_pub = nh.advertise<jsk_rviz_plugins::PictogramArray>("/person_pictgram", 1);


    person_num = 0;

    ros::spin();
}


void PeopleCounter::BoundingBoxCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr &msgs)
{
    int staying_person_num = 0;

    for (size_t i = 0; i < msgs->boxes.size(); i++)
    {
        double min_diff_size = 1e7;
        size_t min_idx = -1;
        geometry_msgs::Point after_pose = msgs->boxes[i].pose.position;       
        for (size_t k = 0; k < bb_array.boxes.size(); k++)
        {
            double diff_size = calculateL2(msgs->boxes[i].pose, bb_array.boxes[k].pose);
            if (diff_size < min_diff_size)
            {
                min_diff_size = diff_size;
                min_idx = k;
                continue;
            }
        }
        std::cout << after_pose.x << ":" << after_pose.y << std::endl;
        double r = sqrt((after_pose.x - x_center) * (after_pose.x - x_center) + ((after_pose.y - y_center) * (after_pose.y - y_center)));
        std::cout << r << " : ";
        if (r < radius*radius) staying_person_num++;
    }
    std::cout << std::endl;
    std::cout << "Staying person : " << staying_person_num << std::endl;
}

void PeopleCounter::MarkerArrayCallback(const visualization_msgs::MarkerArray::ConstPtr &msgs)
{
    int staying_person_num = 0;
    visualization_msgs::MarkerArray temp = (*msgs);
    // std::cout << "Person Number " << person_num << std::endl;
    if (before_people_state.markers.size() == 0) {
        before_people_state = (*msgs);
        for (size_t i = 0; i < before_people_state.markers.size(); i++) 
        {
            before_people_state.markers[i].text = std::to_string(person_num);
            person_num++;
        }
        return;
    } 

    std::vector<size_t> _idx_unknown_object;
    for (size_t i = 0; i < temp.markers.size(); i++)
    {
        double min_diff_size = 1e7;
        size_t min_idx = -1;
        geometry_msgs::Point after_pose = calculateCentroids(temp.markers[i].points);       
        for (size_t k = 0; k < before_people_state.markers.size() ; k++)
        {
            geometry_msgs::Point before_pose = calculateCentroids(before_people_state.markers[k].points);       
            double diff_size = calculateL2(before_pose, after_pose);
            if (diff_size < min_diff_size)
            {
                min_diff_size = diff_size;
                min_idx = k;
                continue;
            }
        }
        
        if (min_diff_size > 2) {
            person_num++;
            temp.markers[i].text = std::to_string(person_num);
        } else if (min_diff_size < 0.5){
            if (before_people_state.markers[min_idx].text == "unknown")
            {
                _idx_unknown_object.push_back(i);
            } else {
                temp.markers[i].text = before_people_state.markers[min_idx].text;
            }
        } else {
            temp.markers[i].text = "unknown";
        }
        std::cout << after_pose.x << ":" << after_pose.y << std::endl;
        double r = sqrt((after_pose.x - x_center) * (after_pose.x - x_center) + ((after_pose.y - y_center) * (after_pose.y - y_center)));
        if (r < radius*radius) staying_person_num++;
        

    }

    visualization_msgs::MarkerArray text_array;
    jsk_rviz_plugins::PictogramArray pict_array;
    for (size_t i = 0; i < temp.markers.size    
(); i++)
    {

        geometry_msgs::Point person_pose= calculateCentroids(temp.markers[i].points);
        visualization_msgs::Marker _object_text;
        _object_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        _object_text.action = 0;
        _object_text.header = temp.markers[i].header;
        _object_text.id = temp.markers[i].id;
        _object_text.color = temp.markers[i].color;
        _object_text.pose.position = person_pose;
        _object_text.text = temp.markers[i].text;
        _object_text.pose.position.z += 0.5;
        _object_text.scale.z = 0.5;


        jsk_rviz_plugins::Pictogram person;
        person.pose.position = person_pose;
        person.pose.position.z = 1.6;
        person.character = "user";
        person.action = jsk_rviz_plugins::Pictogram::JUMP_ONCE;
        // person.ttl = 1;
        person.speed = 1.0;
        person.mode = jsk_rviz_plugins::Pictogram::PICTOGRAM_MODE;
        person.size = 1;
        person.header = temp.markers[i].header;
        person.color.r = 221 / 255.0;
        person.color.g = 255 / 255.0;
        person.color.b = 221 / 255.0;
        person.pose.orientation.w = 0.7;
        person.pose.orientation.x = 0;
        person.pose.orientation.y = -0.7;
        person.pose.orientation.z = 0;

        pict_array.pictograms.push_back(person);
        text_array.markers.push_back(_object_text);    
    }

    
    jsk_rviz_plugins::OverlayText int_data;
    int_data.text = std::to_string(person_num);
    int_data.width = 0.5;
    int_data.height = 0.5;
    int_data.left = 0.5;
    int_data.top = 0.5;
    int_data.fg_color.r = 192/255.0;
    int_data.fg_color.g = 48/255.0;
    int_data.fg_color.b = 192/255.0;
    int_data.fg_color.a = 1;
    int_data.text_size = 12;



    for (size_t k = 0; k < before_people_state.markers.size(); k++)
    {
        size_t _idx;
        std::string label = before_people_state.markers[k].text;
        for (size_t i = 0; i < temp.markers.size(); i++)
        {
            if (temp.markers[i].text == label){

            }
            
        }
    }
    classifier_pub.publish(temp);
    text_pub.publish(text_array);

    person_num_pub.publish(int_data);
    std::cout << "Staying person : " << staying_person_num << std::endl;
}

// void PeopleCounter::ClusterCallback(const visualization_msgs::MarkerArray::ConstPtr &msgs)
// {

// }


void PeopleCounter::LineCallback(const visualization_msgs::MarkerArray::ConstPtr &msgs)
{
    lineArray = (*msgs);
}

double PeopleCounter::calculateL2(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
    double x_diff, y_diff;
    x_diff = b.position.x - a.position.x;
    y_diff = b.position.y - a.position.y;
    return std::sqrt(x_diff * x_diff + y_diff * y_diff);
}

double PeopleCounter::calculateL2(geometry_msgs::Point a, geometry_msgs::Point b)
{
    double x_diff, y_diff;
    x_diff = b.x - a.x;
    y_diff = b.y - a.y;
    return std::sqrt(x_diff * x_diff + y_diff * y_diff);
}


geometry_msgs::Point PeopleCounter::calculateCentroids(std::vector<geometry_msgs::Point> points)
{
    geometry_msgs::Point center;
    float x = 0.0f, y = 0.0f, z = 0.0f;
    for (auto point : points)
    {
        x += point.x;
        y += point.y;
        z += point.z;
    }
    center.x = x / points.size();
    center.y = y / points.size();
    center.z = z / points.size();

    return center;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "people_counter");
    PeopleCounter pc;
    return 0;
}