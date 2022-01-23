#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/Point.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

#include "jsk_recognition_msgs/BoundingBox.h"
#include <visualization_msgs/MarkerArray.h>

#include "jsk_recognition_msgs/BoundingBoxArray.h"

#include <autoware_msgs/CloudCluster.h>
#include <autoware_msgs/CloudClusterArray.h>
#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <map>

#include "svm.h"
#include "people_classifier.h"



ObjectClassifier::ObjectClassifier() 
    : nhPrivate_("~")
{

    nhPrivate_.param<std::string>("input_topic_name", input_topic_name,
                                  "/points_raw");
    std::cout << "input_topic_name : " << input_topic_name << std::endl;
    nhPrivate_.param<std::string>("model_file_name", model_file_name_, "");
    nhPrivate_.param<std::string>("frame_id", frame_id_, "velodyne");
    nhPrivate_.param<std::string>("range_file_name", range_file_name_, "");
    nhPrivate_.param("ratio_depth_tolerance", ratio_depth_tolerance, 0.05);
    std::cout << "ratio_depth_tolerance = " << ratio_depth_tolerance << std::endl;
    nhPrivate_.param("min_tolerance", min_tolerance, 0.1);
    std::cout << "min_tolerance = " << min_tolerance << std::endl;
    nhPrivate_.param("max_tolerance", max_tolerance, 0.5);
    std::cout << "max_tolerance = " << max_tolerance << std::endl;
    nhPrivate_.param("min_cluster_size", min_cluster_size, 65);
    std::cout << "min_cluster_size = " << min_cluster_size << std::endl;
    nhPrivate_.param("max_cluster_size", max_cluster_size, 65);
    std::cout << "max_cluster_size = " << max_cluster_size << std::endl;
    nhPrivate_.param("height_threshold", height_threshold, 0.6);
    std::cout << "height_threshold = " << height_threshold << std::endl;
    nhPrivate_.param<float>("human_probability", human_probability_, 0.8);
    nhPrivate_.param<bool>("human_size_limit", human_size_limit_, false);

    pc_sub = nh_.subscribe<sensor_msgs::PointCloud2>(
        input_topic_name, 5, &ObjectClassifier::PointCloudCallback, this);
    pub_pc = nh_.advertise<sensor_msgs::PointCloud2>("/cluster_points", 1);
    pub_bb =
        nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/cluster_box", 1);

    marker_array_pub_ =
        nhPrivate_.advertise<visualization_msgs::MarkerArray>("markers", 100);

	autoware_detection_pub = nh_.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_objects", 1);
    
    use_svm_model_ = false;
    if ((svm_model_ = svm_load_model(model_file_name_.c_str())) == NULL)
    {
        ROS_WARN("[object3d detector] can not load SVM model, use model-free "
                 "detection.");
    }
    else
    {
        ROS_INFO("[object3d detector] load SVM model from '%s'.",
                 model_file_name_.c_str());
        is_probability_model_ =
            svm_check_probability_model(svm_model_) ? true : false;
        svm_node_ = (struct svm_node *)malloc(
            (FEATURE_SIZE + 1) *
            sizeof(struct svm_node)); // 1 more size for end index (-1)

        // load range file, for more details: https://github.com/cjlin1/libsvm/
        std::fstream range_file;
        range_file.open(range_file_name_.c_str(), std::fstream::in);
        if (!range_file.is_open())
        {
            ROS_WARN("[object3d detector] can not load range file, use model-free "
                     "detection.");
        }
        else
        {
            ROS_INFO("[object3d detector] load SVM range from '%s'.",
                     range_file_name_.c_str());
            std::string line;
            std::vector<std::string> params;
            std::getline(range_file, line);
            std::getline(range_file, line);
            boost::split(params, line, boost::is_any_of(" "));
            x_lower_ = atof(params[0].c_str());
            x_upper_ = atof(params[1].c_str());
            int i = 0;
            while (std::getline(range_file, line))
            {
                boost::split(params, line, boost::is_any_of(" "));
                svm_scale_range_[i][0] = atof(params[1].c_str());
                svm_scale_range_[i][1] = atof(params[2].c_str());
                i++;
                // std::cerr << i << " " <<  svm_scale_range_[i][0] << " " <<
                // svm_scale_range_[i][1] << std::endl;
            }
            use_svm_model_ = true;
        }
    }
}

ObjectClassifier::~ObjectClassifier()
{
    if (use_svm_model_)
    {
        svm_free_and_destroy_model(&svm_model_);
        free(svm_node_);
    }
}

double ObjectClassifier::ComputeTolerance(const pcl::PointXYZI &point)
{
    /*センサからの距離（depth）*/
    double depth =
        sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    double tolerance = ratio_depth_tolerance * depth; //距離に比例
    if (tolerance < min_tolerance)
        tolerance = min_tolerance;
    if (tolerance > max_tolerance)
        tolerance = max_tolerance;

    return tolerance;
}

void ObjectClassifier::PublishCloud()
{
    ROS_INFO_STREAM("PublishCloud");
    pcl::PointCloud<pcl::PointXYZRGB> result;
    jsk_recognition_msgs::BoundingBoxArray bb_array;
    autoware_msgs::DetectedObjectArray ttdc_object_array;
    ros::Time now_time = ros::Time::now();

    std::cout << "Size : " << clusters.size() << std::endl;
    if (!clusters.empty())
    {
        int i = 1;
        float colors[6][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {0, 255, 255}, {255, 0, 255}};
        for (auto cluster : clusters)
        {

            std::cout << "cluster" << i << " " << cluster->points.size() << std::endl;
            std::cout << "color: R " << colors[i - 1][0] << " G " << colors[i - 1][1]
                      << " B " << colors[i - 1][2] << std::endl;
            pcl::PointCloud<pcl::PointXYZRGB> _cloud;
            pcl::copyPointCloud(*cluster, _cloud);

            geometry_msgs::Vector3 cluster_size = calculateSize(cluster);
            for (size_t j = 0; j < _cloud.points.size(); j++)
            {
                _cloud.points[j].r = colors[i - 1][0];
                _cloud.points[j].g = colors[i - 1][1];
                _cloud.points[j].b = colors[i - 1][2];
            }


            result += (_cloud);

            
            
            /*-------Bounding Box------*/
            jsk_recognition_msgs::BoundingBox bb;
            bb.header.frame_id = "velodyne";
            bb.header.stamp = now_time;
            bb.pose.position = Centroids(cluster);
            bb.dimensions = cluster_size;
            bb.label = i;
            bb_array.boxes.push_back(bb);
            /*-------------------------*/

            i++;

            /*-------TTDC---------*/
            autoware_msgs::DetectedObject ttdc_object;
            sensor_msgs::PointCloud2::Ptr _cloud2(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(_cloud, *_cloud2);
            ttdc_object.pose.position = Centroids(cluster);
            ttdc_object.pointcloud = (*_cloud2);

            ttdc_object.header.frame_id = "velodyne";
            ttdc_object.header.stamp = now_time;

            ttdc_object_array.objects.push_back(ttdc_object);
            /*--------------------*/
        }

        ttdc_object_array.header.frame_id = "velodyne";
        ttdc_object_array.header.stamp = now_time;
        pub_ttdc_object.publish(ttdc_object_array);

        std::cout << "Point Size" << result.points.size() << std::endl;
        sensor_msgs::PointCloud2::Ptr tempROSMsg(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(result, *tempROSMsg);
        tempROSMsg->header.frame_id = "velodyne";
        tempROSMsg->header.stamp = now_time;
        pub_pc.publish(tempROSMsg);

        bb_array.header.frame_id = "velodyne";
        bb_array.header.stamp = now_time;
        pub_bb.publish(bb_array);
    }
}

void ObjectClassifier::PublishCluster(std::vector<size_t> idx_list, std::map<size_t, geometry_msgs::Polygon> pol_map)
{
    autoware_msgs::DetectedObjectArray _person_object_array;
    for (size_t _idx : idx_list)
    {
        sensor_msgs::PointCloud2::Ptr _cloud2(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*clusters[_idx], *_cloud2);

       

        autoware_msgs::DetectedObject person_object;
        person_object.label = "unknown";
        person_object.score = 1;
        person_object.pose.position = Centroids(clusters[_idx]);
        person_object.convex_hull.polygon = pol_map[_idx];
        person_object.pointcloud = (*_cloud2);
        person_object.header.frame_id = "velodyne";
        person_object.header.stamp = ros::Time::now();
        person_object.valid = true;


        _person_object_array.objects.push_back(person_object);        
    }

    autoware_detection_pub.publish(_person_object_array);
}

geometry_msgs::Point ObjectClassifier::Centroids(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    geometry_msgs::Point centroid;
    centroid.x = 0;
    centroid.y = 0;
    centroid.z = 0;

    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        centroid.x += cloud->points[i].x;
        centroid.y += cloud->points[i].y;
        centroid.z += cloud->points[i].z;
    }

    centroid.x /= cloud->points.size();
    centroid.y /= cloud->points.size();
    centroid.z /= cloud->points.size();
    return centroid;
}

void ObjectClassifier::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msgs)
{
    pcl::fromROSMsg(*msgs, *cloud);
    clusters.clear();
    Clustering();
    PublishCloud();
    features_.clear();

    for (auto cluster : clusters)
    {
        Eigen::Vector4f min, max, centroid;
        pcl::getMinMax3D(*cluster, min, max);
        pcl::compute3DCentroid(*cluster, centroid);

        if (human_size_limit_ &&
                    (max[0] - min[0] < 0.2 || max[0] - min[0] > 1.0 ||
                     max[1] - min[1] < 0.2 || max[1] - min[1] > 1.0 ||
                     max[2] - min[2] < 0.5 || max[2] - min[2] > 2.0))
                    continue;

        Feature f;
        extractFeature(cluster, f, min, max, centroid);
        features_.push_back(f);
    }
    classify();
}

void ObjectClassifier::classify()
{
    visualization_msgs::MarkerArray marker_array;
    std::vector<size_t> _idx_list;
    std::map<size_t, geometry_msgs::Polygon> pol_map;
    for (std::vector<Feature>::iterator it = features_.begin();
         it != features_.end(); ++it)
    {
        if (use_svm_model_)
        {
            saveFeature(*it, svm_node_);
            // std::cerr << "test_id = " << it->id << ", number_points = " <<
            // it->number_points << ", min_distance = " << it->min_distance <<
            // std::endl;

            // scale data
            for (int i = 0; i < FEATURE_SIZE; i++)
            {
                if (svm_scale_range_[i][0] ==
                    svm_scale_range_[i][1]) // skip single-valued attribute
                    continue;
                if (svm_node_[i].value == svm_scale_range_[i][0])
                    svm_node_[i].value = x_lower_;
                else if (svm_node_[i].value == svm_scale_range_[i][1])
                    svm_node_[i].value = x_upper_;
                else
                    svm_node_[i].value =
                        x_lower_ + (x_upper_ - x_lower_) *
                                       (svm_node_[i].value - svm_scale_range_[i][0]) /
                                       (svm_scale_range_[i][1] - svm_scale_range_[i][0]);
            }

            // predict
            if (is_probability_model_)
            {
                double prob_estimates[svm_model_->nr_class];
                svm_predict_probability(svm_model_, svm_node_, prob_estimates);
                if (prob_estimates[0] < human_probability_)
                    continue;
            }
            else
            {
                if (svm_predict(svm_model_, svm_node_) != 1)
                    continue;
            }
        }

        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = frame_id_;
        // marker.ns = "object3d";
        marker.id = it - features_.begin();
        marker.type = visualization_msgs::Marker::LINE_LIST;
        geometry_msgs::Point p[24];
        p[0].x = it->max[0];
        p[0].y = it->max[1];
        p[0].z = it->max[2];
        p[1].x = it->min[0];
        p[1].y = it->max[1];
        p[1].z = it->max[2];
        p[2].x = it->max[0];
        p[2].y = it->max[1];
        p[2].z = it->max[2];
        p[3].x = it->max[0];
        p[3].y = it->min[1];
        p[3].z = it->max[2];
        p[4].x = it->max[0];
        p[4].y = it->max[1];
        p[4].z = it->max[2];
        p[5].x = it->max[0];
        p[5].y = it->max[1];
        p[5].z = it->min[2];
        p[6].x = it->min[0];
        p[6].y = it->min[1];
        p[6].z = it->min[2];
        p[7].x = it->max[0];
        p[7].y = it->min[1];
        p[7].z = it->min[2];
        p[8].x = it->min[0];
        p[8].y = it->min[1];
        p[8].z = it->min[2];
        p[9].x = it->min[0];
        p[9].y = it->max[1];
        p[9].z = it->min[2];
        p[10].x = it->min[0];
        p[10].y = it->min[1];
        p[10].z = it->min[2];
        p[11].x = it->min[0];
        p[11].y = it->min[1];
        p[11].z = it->max[2];
        p[12].x = it->min[0];
        p[12].y = it->max[1];
        p[12].z = it->max[2];
        p[13].x = it->min[0];
        p[13].y = it->max[1];
        p[13].z = it->min[2];
        p[14].x = it->min[0];
        p[14].y = it->max[1];
        p[14].z = it->max[2];
        p[15].x = it->min[0];
        p[15].y = it->min[1];
        p[15].z = it->max[2];
        p[16].x = it->max[0];
        p[16].y = it->min[1];
        p[16].z = it->max[2];
        p[17].x = it->max[0];
        p[17].y = it->min[1];
        p[17].z = it->min[2];
        p[18].x = it->max[0];
        p[18].y = it->min[1];
        p[18].z = it->max[2];
        p[19].x = it->min[0];
        p[19].y = it->min[1];
        p[19].z = it->max[2];
        p[20].x = it->max[0];
        p[20].y = it->max[1];
        p[20].z = it->min[2];
        p[21].x = it->min[0];
        p[21].y = it->max[1];
        p[21].z = it->min[2];
        p[22].x = it->max[0];
        p[22].y = it->max[1];
        p[22].z = it->min[2];
        p[23].x = it->max[0];
        p[23].y = it->min[1];
        p[23].z = it->min[2];
        for (int i = 0; i < 24; i++)
            marker.points.push_back(p[i]);
        marker.scale.x = 0.02;
        marker.color.a = 1.0;
        if (!use_svm_model_)
        {
            marker.color.r = 0.0;
            marker.color.g = 0.5;
            marker.color.b = 1.0;
        }
        else
        {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.5;
        }

        marker.lifetime = ros::Duration(0.1);
        marker_array.markers.push_back(marker);

        size_t idx = std::distance(features_.begin(), it);
        _idx_list.push_back(idx);

        geometry_msgs::Polygon _pol;
        for (auto point: p) {
            geometry_msgs::Point32 _point32;
            _point32.x = point.x;
            _point32.y = point.y;
            _point32.z = point.z;
            _pol.points.push_back(_point32);
        }
        pol_map[idx] = _pol;
    }

    PublishCluster(_idx_list, pol_map);

    if (marker_array.markers.size())
    {
        marker_array_pub_.publish(marker_array);
    }


}
void ObjectClassifier::saveFeature(Feature &f, struct svm_node *x)
{
    x[0].index = 1;
    x[0].value = f.number_points; // libsvm indices start at 1
    x[1].index = 2;
    x[1].value = f.min_distance;
    x[2].index = 3;
    x[2].value = f.covariance_3d(0, 0);
    x[3].index = 4;
    x[3].value = f.covariance_3d(0, 1);
    x[4].index = 5;
    x[4].value = f.covariance_3d(0, 2);
    x[5].index = 6;
    x[5].value = f.covariance_3d(1, 1);
    x[6].index = 7;
    x[6].value = f.covariance_3d(1, 2);
    x[7].index = 8;
    x[7].value = f.covariance_3d(2, 2);
    x[8].index = 9;
    x[8].value = f.moment_3d(0, 0);
    x[9].index = 10;
    x[9].value = f.moment_3d(0, 1);
    x[10].index = 11;
    x[10].value = f.moment_3d(0, 2);
    x[11].index = 12;
    x[11].value = f.moment_3d(1, 1);
    x[12].index = 13;
    x[12].value = f.moment_3d(1, 2);
    x[13].index = 14;
    x[13].value = f.moment_3d(2, 2);
    // for(int i = 0; i < 9; i++) {
    //   x[i+14].index = i+15;
    //   x[i+14].value = f.partial_covariance_2d[i];
    // }
    // for(int i = 0; i < 98; i++) {
    // 	x[i+23].index = i+24;
    // 	x[i+23].value = f.histogram_main_2d[i];
    // }
    // for(int i = 0; i < 45; i++) {
    // 	x[i+121].index = i+122;
    // 	x[i+121].value = f.histogram_second_2d[i];
    // }
    for (int i = 0; i < 20; i++)
    {
        x[i + 14].index = i + 15;
        x[i + 14].value = f.slice[i];
    }
    for (int i = 0; i < 27; i++)
    {
        x[i + 34].index = i + 35;
        x[i + 34].value = f.intensity[i];
    }
    x[FEATURE_SIZE].index = -1;

    // for(int i = 0; i < FEATURE_SIZE; i++) {
    //   std::cerr << x[i].index << ":" << x[i].value << " ";
    //   std::cerr << std::endl;
    // }
}

void ObjectClassifier::Clustering(void)
{
    double time_start = ros::Time::now().toSec();

    /*search config*/
    /*kd-treeクラスを宣言*/
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    /*探索する点群をinput*/
    kdtree.setInputCloud(cloud);
      max_cluster_size = cloud->points.size();
    /*objects*/
    std::vector<pcl::PointIndices> cluster_indices;
    std::vector<bool> processed(cloud->points.size(), false);
    std::vector<int> nn_indices;
    std::vector<float> nn_distances;
    /*clustering*/
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        if (processed[i])
            continue; //既に分類されているかチェック
        /*set seed（シード点を設定）*/
        std::vector<int> seed_queue;
        int sq_idx = 0;
        seed_queue.push_back(i);
        processed[i] = true;
        /*clustering*/
        while (sq_idx < seed_queue.size())
        { //探索しきるまでループ
            /*search*/
            double tolerance = ComputeTolerance(cloud->points[seed_queue[sq_idx]]);
            int ret = kdtree.radiusSearch(cloud->points[seed_queue[sq_idx]],
                                          tolerance, nn_indices, nn_distances);
            if (ret == -1)
            {
                PCL_ERROR("[pcl::extractEuclideanClusters] Received error code -1 from "
                          "radiusSearch\n");
                exit(0);
            }
            /*check*/
            for (size_t j = 0; j < nn_indices.size(); ++j)
            {
                /*//既に分類されているかチェック*/
                if (nn_indices[j] == -1 || processed[nn_indices[j]])
                    continue;
                /*カスタム条件でチェック*/
                if (CustomCondition(cloud->points[seed_queue[sq_idx]],
                                    cloud->points[nn_indices[j]], nn_distances[j]))
                {
                    seed_queue.push_back(nn_indices[j]);
                    processed[nn_indices[j]] = true;
                }
            }
            sq_idx++;
        }
        /*judge（クラスタのメンバ数が条件を満たしているか）*/
        if (seed_queue.size() >= min_cluster_size &&
            seed_queue.size() <= max_cluster_size)
        {
            pcl::PointIndices tmp_indices;
            tmp_indices.indices = seed_queue;
            cluster_indices.push_back(tmp_indices);
        }
    }

    /*extraction（クラスタごとに点群を分割）*/
    pcl::ExtractIndices<pcl::PointXYZI> ei;
    ei.setInputCloud(cloud);
    ei.setNegative(false);
    for (size_t i = 0; i < cluster_indices.size(); i++)
    {
        /*extract*/
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_clustered_points(
            new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointIndices::Ptr tmp_clustered_indices(new pcl::PointIndices);
        *tmp_clustered_indices = cluster_indices[i];
        ei.setIndices(tmp_clustered_indices);
        ei.filter(*tmp_clustered_points);
        /*input*/
        clusters.push_back(tmp_clustered_points);
    }
}



bool ObjectClassifier::CustomCondition(const pcl::PointXYZI &seed_point,
                                       const pcl::PointXYZI &candidate_point,
                                       float squared_distance)
{
    return true;
}

geometry_msgs::Vector3 ObjectClassifier::calculateSize(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    geometry_msgs::Vector3 vec3f;
    double max_z = -1e7;
    double min_z = 1e7;
    double max_y = -1e7;
    double min_y = 1e7;
    double max_x = -1e7;
    double min_x = 1e7;
    
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        if (max_z < cloud->points[i].z)
            max_z = cloud->points[i].z;
        if (min_z >= cloud->points[i].z)
            min_z = cloud->points[i].z;
        if (max_y < cloud->points[i].y)
            max_y = cloud->points[i].y;
        if (min_y >= cloud->points[i].y)
            min_y = cloud->points[i].y;
        if (max_x < cloud->points[i].x)
            max_x = cloud->points[i].x;
        if (min_x >= cloud->points[i].x)
            min_x = cloud->points[i].x;
    }

    vec3f.x = max_x - min_x;
    vec3f.y = max_y - min_y;
    vec3f.z = max_z - min_z;
    if (vec3f.x <= 0.0f )
        vec3f.x = 1.0f;
    if (vec3f.y <= 0.0f )
        vec3f.y = 1.0f;
    return vec3f;
}



void ObjectClassifier::extractFeature(pcl::PointCloud<pcl::PointXYZI>::Ptr pc,
                                      Feature &f, Eigen::Vector4f &min,
                                      Eigen::Vector4f &max,
                                      Eigen::Vector4f &centroid)
{
    f.centroid = centroid;
    f.min = min;
    f.max = max;

    if (use_svm_model_)
    {
        // f1: Number of points included the cluster.
        f.number_points = pc->size();
        // f2: The minimum distance to the cluster.
        f.min_distance = FLT_MAX;
        float d2; // squared Euclidean distance
        for (int i = 0; i < pc->size(); i++)
        {
            d2 = pc->points[i].x * pc->points[i].x +
                 pc->points[i].y * pc->points[i].y +
                 pc->points[i].z * pc->points[i].z;
            if (f.min_distance > d2)
                f.min_distance = d2;
        }
        // f.min_distance = sqrt(f.min_distance);

        pcl::PCA<pcl::PointXYZI> pca;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_projected(
            new pcl::PointCloud<pcl::PointXYZI>);
        pca.setInputCloud(pc);
        pca.project(*pc, *pc_projected);
        // f3: 3D covariance matrix of the cluster.
        pcl::computeCovarianceMatrixNormalized(*pc_projected, centroid,
                                               f.covariance_3d);
        // f4: The normalized moment of inertia tensor.
        computeMomentOfInertiaTensorNormalized(*pc_projected, f.moment_3d);
        // Navarro et al. assume that a pedestrian is in an upright position.
        // pcl::PointCloud<pcl::PointXYZI>::Ptr main_plane(new
        // pcl::PointCloud<pcl::PointXYZI>), secondary_plane(new
        // pcl::PointCloud<pcl::PointXYZI>); computeProjectedPlane(pc,
        // pca.getEigenVectors(), 2, centroid, main_plane);
        // computeProjectedPlane(pc, pca.getEigenVectors(), 1, centroid,
        // secondary_plane);
        // f5: 2D covariance matrix in 3 zones, which are the upper half, and the
        // left and right lower halves.
        // compute3ZoneCovarianceMatrix(main_plane, pca.getMean(),
        // f.partial_covariance_2d);
        // f6 and f7
        // computeHistogramNormalized(main_plane, 7, 14, f.histogram_main_2d);
        // computeHistogramNormalized(secondary_plane, 5, 9, f.histogram_second_2d);
        // f8
        computeSlice(pc, 10, f.slice);
        // f9
        computeIntensity(pc, 25, f.intensity);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "people_classifier");
    ObjectClassifier oc;
    ros::spin();
    return 0;
}