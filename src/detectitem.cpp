#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


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
#include <pcl_ros/transforms.h>

#include "pc_tool.h"

class DetectItem
{
    public:
        DetectItem();
        pcl::PointCloud<pcl::PointXYZI> difference_extraction(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_base, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_test, double resolution);
        pcl::PointCloud<pcl::PointXYZI> difference_extraction(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_base, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_list, float resolution);
        pcl::PointCloud<pcl::PointXYZI> planeRemoval(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
        void Callback(const sensor_msgs::PointCloud2ConstPtr &msg);
        void IMUCallback(const sensor_msgs::ImuConstPtr &msgs);
        
        
    private:
        // Node Handler
        ros::NodeHandle nh;
        ros::NodeHandle nhPrivate;

        ros::Publisher pub_pc;
        ros::Publisher plane_remove_pub;
        ros::Subscriber sub_pc;
        ros::Subscriber sub_imu;

        std::string compare_pcd_file, input_topic, output_topic, imu_topic;
        std::string map_frame, velodyne_frame;
        float resolution, map_resolution;
        int maxSize, saveCnt;
        double finishRemovePlaneRatio, removeThreshold;
        float angular_threshold, defaultResolution, angular_bias, imu_angular_z;
        tf::Quaternion tf_imu_rotation;

        bool first_callback_imu;

        sensor_msgs::Imu imu_data;
        tf2_ros::Buffer tfBuffer;
        geometry_msgs::TransformStamped transform_stamped;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_base{new pcl::PointCloud<pcl::PointXYZI>};
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_base{new pcl::PointCloud<pcl::PointXYZI>};

        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> frame_pcd;
};


DetectItem::DetectItem()
    : nhPrivate("~")
{
    input_topic = "/points_no_ground";
    output_topic = "/points_no_wall";

    map_frame = "map";
    velodyne_frame = "velodyne";
    imu_topic = "/imu/data";
    nhPrivate.getParam("resolution", resolution);
    nhPrivate.getParam("map_resolution", map_resolution);
    nhPrivate.getParam("input_topic", input_topic);
    nhPrivate.getParam("compare_pcd_file", compare_pcd_file);
    nhPrivate.getParam("output_topic", output_topic);
    nhPrivate.getParam("maxSize", maxSize);
    nhPrivate.getParam("finishRemovePlaneRatio", finishRemovePlaneRatio);
    nhPrivate.getParam("removeThreshold", removeThreshold);
    nhPrivate.getParam("map_frame", map_frame);
    nhPrivate.param("angular_threshold", angular_threshold, 0.1f);

    first_callback_imu = true;

    angular_bias = 0.0f;
    plane_remove_pub = nh.advertise<sensor_msgs::PointCloud2>("/remove_pc", 1);
    pub_pc = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 1);

    sub_imu = nh.subscribe(imu_topic, 1, &DetectItem::IMUCallback, this);
    sub_pc = nh.subscribe(input_topic, 1, &DetectItem::Callback, this);

    tf2_ros::TransformListener listener(tfBuffer);

    saveCnt = 0;

    pcl::io::loadPCDFile(compare_pcd_file, *cloud_base);
    pcl::io::loadPCDFile(compare_pcd_file, *map_base);

    ros::spin();
}


// void DetectItem::Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
// {

//     // Velodyne -> Map
//     pcl::PointCloud<pcl::PointXYZI> cloud;
//     while (true) {
//         try
//         {
//             ROS_INFO_STREAM("Get TF");
//             transform_stamped = tfBuffer.lookupTransform(map_frame, msg->header.frame_id, ros::Time(0));
//             break;
//         }
//         catch (tf::TransformException ex)
//         {
//             // ROS_ERROR("%s", ex.what());
//             ros::Duration(0.1).sleep();
//         }
//     }
//     sensor_msgs::PointCloud2 transformed_cloud;
//     Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
//     pcl_ros::transformPointCloud(mat, *msg, transformed_cloud);
//     // SensorMsg::PointCloud2 -> pcl::PointCloud
//     pcl::fromROSMsg(transformed_cloud, cloud);

//     // cloud
//     // Now PointCloud frame : map

//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(cloud));
//     pcl::PointCloud<pcl::PointXYZI>::Ptr time_subst_pc(new pcl::PointCloud<pcl::PointXYZI>);
//     pcl::PointCloud<pcl::PointXYZI>::Ptr map_subst_pc(new pcl::PointCloud<pcl::PointXYZI>);

//     // Substraction
//     if (frame_pcd.size() > 0) 
//     {
//         pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>);
//         temp = cloud_ptr;

//         for (auto pcd : frame_pcd)
//         {
//             *temp = DetectItem::difference_extraction(pcd, temp, resolution);
//         }
//         *map_subst_pc = DetectItem::difference_extraction(map_base, temp, map_resolution);
//         // *temp = DetectItem::difference_extraction(temp, frame_pcd, resolution);
//     } else {
//         *time_subst_pc = DetectItem::difference_extraction(cloud_base, cloud_ptr, resolution);
//         // *map_subst_pc = DetectItem::difference_extraction(cloud_base, cloud_ptr, resolution);
//         *map_subst_pc = DetectItem::difference_extraction(map_base, time_subst_pc, map_resolution);
//         // DetectItem::planeRemoval(map_subst_pc, removeThreshold);
//     }




//     // pcl::PointCloud -> SensorMsgs::PointCloud2
//     sensor_msgs::PointCloud2::Ptr pc2_cloud(new sensor_msgs::PointCloud2);
//     pcl::toROSMsg(*map_subst_pc, *pc2_cloud);
//     pc2_cloud->header.frame_id = map_frame;
//     pc2_cloud->header.stamp = ros::Time::now();


//     // Map -> Velodyne
//     sensor_msgs::PointCloud2::Ptr pc2_cloud_transformed(new sensor_msgs::PointCloud2);
//     // pcl_ros::transformPointCloud(map_frame, transform, *recent_cloud, transformed_cloud);
//     Eigen::Matrix4f mat2 = tf2::transformToEigen(transform_stamped.transform).matrix().inverse().cast<float>();
//     pcl_ros::transformPointCloud(mat2, *pc2_cloud, *pc2_cloud_transformed);
//     pc2_cloud_transformed->header.frame_id = velodyne_frame;
//     pc2_cloud_transformed->header.stamp = ros::Time::now();

//     pub_pc.publish(pc2_cloud_transformed);

//     // Save PCD
//     if (frame_pcd.size() > maxSize) {
//         // frame_pcd.pop_front();
//         frame_pcd.erase(frame_pcd.begin());
//     }
    
//     // frame_pcd.push_back(cloud_ptr);
//     if (saveCnt == maxSize)
//     {
//         cloud_base = cloud_ptr;
//     }
//     saveCnt++;

    // マップ差分をしたものを保存したのを引く
// }

void DetectItem::IMUCallback(const sensor_msgs::ImuConstPtr &msgs)
{
    float z_angular_velocity = std::abs(msgs->angular_velocity.z);
    if (z_angular_velocity > angular_threshold)
    {
        angular_bias = 0.1f;
    } else {
        angular_bias = 0.0f;
    }
    // angular_bias = z_angular_velocity / 2.0f;
    imu_angular_z = z_angular_velocity;
    if (first_callback_imu) {
        first_callback_imu = false;
        imu_data = (*msgs);
        return;
    }
    ros::Time time_imu_now = msgs->header.stamp;
    ros::Time time_imu_last = imu_data.header.stamp;
    double dt;
    try {
        dt = (time_imu_now - time_imu_last).toSec();
    } catch(std::runtime_error& ex) {
		ROS_ERROR("Exception: [%s]", ex.what());
	}
    
    double delta_r = (msgs->angular_velocity.x + imu_data.angular_velocity.x)*dt/2.0;
    double delta_p = (msgs->angular_velocity.y + imu_data.angular_velocity.y)*dt/2.0;
    double delta_y = (msgs->angular_velocity.z + imu_data.angular_velocity.z)*dt/2.0;
    // tf_imu_rotation = tf::createQuaternionFromRPY(delta_r, delta_p, delta_y);
    tf_imu_rotation = tf::createQuaternionFromYaw(-delta_y);
    std::cout << "timestamp : " << msgs->header.stamp << std::endl;
    
    imu_data = (*msgs);
}

void DetectItem::Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{

    // Velodyne -> Map
    pcl::PointCloud<pcl::PointXYZI> cloud;
    while (ros::ok()) {
        try
        {
            transform_stamped = tfBuffer.lookupTransform(map_frame, msg->header.frame_id, ros::Time(0));
            break;
        }
        catch (tf::TransformException ex)
        {
            // ROS_ERROR("%s", ex.what());
            ros::Duration(0.1).sleep();
        }
        catch (ros::Exception a)
        {
            ROS_ERROR("Exception : ", a);
            ros::shutdown();
            return;
        }
    }
    sensor_msgs::PointCloud2 transformed_cloud;
    Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
    pcl_ros::transformPointCloud(mat, *msg, transformed_cloud);
    // SensorMsg::PointCloud2 -> pcl::PointCloud
    pcl::fromROSMsg(transformed_cloud, cloud);

    // cloud
    // Now PointCloud frame : map

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(cloud));
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr_store(new pcl::PointCloud<pcl::PointXYZI>(cloud));
    pcl::PointCloud<pcl::PointXYZI>::Ptr remove_plane_pcd(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr time_subst_pc(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_subst_pc(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transfromed_cloud_imu(new pcl::PointCloud<pcl::PointXYZI>);

    
    // Substraction
    if (frame_pcd.size() > 0) 
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>);
        temp = cloud_ptr;

        for (auto pcd : frame_pcd)
        {
            *temp = DetectItem::difference_extraction(pcd, temp, resolution);
        }
        // *temp = DetectItem::difference_extraction(temp, frame_pcd, resolution);
        *map_subst_pc = DetectItem::difference_extraction(map_base, temp, map_resolution);
    } else {
        // *remove_plane_pcd = DetectItem::planeRemoval(transformed_cloud_imu);
        // std::cout << "Remove(originals) PointCloud : " << cloud_ptr->points.size() << std::endl;
        // std::cout << "Remove PointCloud : " << remove_plane_pcd->points.size() << std::endl;
        // std::cout << "resolution : " <<  map_resolution + angular_bias << std::endl; 
        // *time_subst_pc = DetectItem::difference_extraction(cloud_base, remove_plane_pcd, resolution);
        if (imu_angular_z > angular_threshold)
        {
            // std::cout << "delta_y : " << tf_imu_rotation.getAxis().getY() << std::endl;
            tf::Transform imutotf;
            imutotf.setRotation(tf_imu_rotation);
            pcl_ros::transformPointCloud(*cloud_ptr, *transfromed_cloud_imu, imutotf);
            *time_subst_pc = DetectItem::difference_extraction(cloud_base, transfromed_cloud_imu, resolution);
        } else {
            *time_subst_pc = DetectItem::difference_extraction(cloud_base, cloud_ptr, resolution);
        }

        // *time_subst_pc = DetectItem::difference_extraction(cloud_base, cloud_ptr, resolution);
        *map_subst_pc = DetectItem::difference_extraction(map_base, time_subst_pc, map_resolution + angular_bias);
    }




    // pcl::PointCloud -> SensorMsgs::PointCloud2
    sensor_msgs::PointCloud2::Ptr pc2_cloud(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*map_subst_pc, *pc2_cloud);
    pc2_cloud->header.frame_id = map_frame;
    pc2_cloud->header.stamp = ros::Time::now();


    // Map -> Velodyne
    sensor_msgs::PointCloud2::Ptr pc2_cloud_transformed(new sensor_msgs::PointCloud2);
    // pcl_ros::transformPointCloud(map_frame, transform, *recent_cloud, transformed_cloud);
    Eigen::Matrix4f mat2 = tf2::transformToEigen(transform_stamped.transform).matrix().inverse().cast<float>();
    pcl_ros::transformPointCloud(mat2, *pc2_cloud, *pc2_cloud_transformed);
    pc2_cloud_transformed->header.frame_id = velodyne_frame;
    pc2_cloud_transformed->header.stamp = ros::Time::now();

    pub_pc.publish(pc2_cloud_transformed);

    // Save PCD
    if (frame_pcd.size() > maxSize) {
        // frame_pcd.pop_front();
        frame_pcd.erase(frame_pcd.begin());
    }
    
    // frame_pcd.push_back(cloud_ptr);
    if (saveCnt == maxSize)
    {
        cloud_base = cloud_ptr_store;
    }
    

    saveCnt++;
}



pcl::PointCloud<pcl::PointXYZI> DetectItem::difference_extraction(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_base, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_test, double resolution = 0.01)
{
    //cloud_baseは元となる点群
    //cloud_testは比較対象の点群
    //cloud_diffは比較の結果差分とされた点群

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
        cloud_diff->points[i].intensity = cloud_test->points[newPointIdxVector[i]].intensity;
        n++;
    }

    //差分点群のサイズの再設定
    cloud_diff->width = n;
    cloud_diff->height = 1;
    cloud_diff->points.resize(cloud_diff->width * cloud_diff->height);

    return *cloud_diff;
}


pcl::PointCloud<pcl::PointXYZI> DetectItem::planeRemoval(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) 
{
    //　平面検出（pclのチュートリアル通り）
	pcl::SACSegmentation<pcl::PointXYZI> seg;
	seg.setOptimizeCoefficients(true);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	seg.setInputCloud(cloud);
    seg.setOptimizeCoefficients (true);  
	seg.setModelType(pcl::SACMODEL_PLANE); //モデル
	seg.setMethodType(pcl::SAC_RANSAC);	//検出手法
	seg.setMaxIterations(200);
	seg.setDistanceThreshold(removeThreshold); //閾値
	seg.segment(*inliers, *coefficients);

	// 平面除去を終わらせるかどうか：検出した平面が，前の平面除去した状態の点群のfinishRemovePlaneRatioで指定された割合未満の点群サイズであれば，それは平面でないとみなして点群除去処理を終了（finishRemovePlaneRatioは0.0~1.0:環境に合わせてください）
	// if (inliers->indices.size() < cloud->points.size() * finishRemovePlaneRatio){
	// 	return *cloud;
	// }

	// 平面除去
	pcl::ExtractIndices<pcl::PointXYZI> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(true); // true にすると平面を除去、false にすると平面以外を除去
	extract.filter(*cloud);
    
    return *cloud;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "DetectionMovingItem");
    DetectItem di;
    return 0;
}