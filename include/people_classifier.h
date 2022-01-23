#pragma once
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

typedef struct feature
{
    /*** for visualization ***/
    Eigen::Vector4f centroid;
    Eigen::Vector4f min;
    Eigen::Vector4f max;
    /*** for classification ***/
    int number_points;
    float min_distance;
    Eigen::Matrix3f covariance_3d;
    Eigen::Matrix3f moment_3d;
    // float partial_covariance_2d[9];
    // float histogram_main_2d[98];
    // float histogram_second_2d[45];
    float slice[20];
    float intensity[27];
} Feature;
static const int FEATURE_SIZE = 61;

void computeMomentOfInertiaTensorNormalized(pcl::PointCloud<pcl::PointXYZI> &pc,
                                            Eigen::Matrix3f &moment_3d)
{
    moment_3d.setZero();
    for (size_t i = 0; i < pc.size(); i++)
    {
        moment_3d(0, 0) += pc[i].y * pc[i].y + pc[i].z * pc[i].z;
        moment_3d(0, 1) -= pc[i].x * pc[i].y;
        moment_3d(0, 2) -= pc[i].x * pc[i].z;
        moment_3d(1, 1) += pc[i].x * pc[i].x + pc[i].z * pc[i].z;
        moment_3d(1, 2) -= pc[i].y * pc[i].z;
        moment_3d(2, 2) += pc[i].x * pc[i].x + pc[i].y * pc[i].y;
    }
    moment_3d(1, 0) = moment_3d(0, 1);
    moment_3d(2, 0) = moment_3d(0, 2);
    moment_3d(2, 1) = moment_3d(1, 2);
}

/* Main plane is formed from the maximum and middle eigenvectors.
 * Secondary plane is formed from the middle and minimum eigenvectors.
 */
void computeProjectedPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr pc,
                           Eigen::Matrix3f &eigenvectors, int axe,
                           Eigen::Vector4f &centroid,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr plane)
{
    Eigen::Vector4f coefficients;
    coefficients[0] = eigenvectors(0, axe);
    coefficients[1] = eigenvectors(1, axe);
    coefficients[2] = eigenvectors(2, axe);
    coefficients[3] = 0;
    coefficients[3] = -1 * coefficients.dot(centroid);
    for (size_t i = 0; i < pc->size(); i++)
    {
        float distance_to_plane =
            coefficients[0] * pc->points[i].x + coefficients[1] * pc->points[i].y +
            coefficients[2] * pc->points[i].z + coefficients[3];
        pcl::PointXYZI p;
        p.x = pc->points[i].x - distance_to_plane * coefficients[0];
        p.y = pc->points[i].y - distance_to_plane * coefficients[1];
        p.z = pc->points[i].z - distance_to_plane * coefficients[2];
        plane->points.push_back(p);
    }
}

/* Upper half, and the left and right lower halves of a pedestrian. */
void compute3ZoneCovarianceMatrix(pcl::PointCloud<pcl::PointXYZI>::Ptr plane,
                                  Eigen::Vector4f &mean,
                                  float *partial_covariance_2d)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr zone_decomposed[3];
    for (int i = 0; i < 3; i++)
        zone_decomposed[i].reset(new pcl::PointCloud<pcl::PointXYZI>);
    for (size_t i = 0; i < plane->size(); i++)
    {
        if (plane->points[i].z >= mean(2))
        { // upper half
            zone_decomposed[0]->points.push_back(plane->points[i]);
        }
        else
        {
            if (plane->points[i].y >= mean(1)) // left lower half
                zone_decomposed[1]->points.push_back(plane->points[i]);
            else // right lower half
                zone_decomposed[2]->points.push_back(plane->points[i]);
        }
    }

    Eigen::Matrix3f covariance;
    Eigen::Vector4f centroid;
    for (int i = 0; i < 3; i++)
    {
        pcl::compute3DCentroid(*zone_decomposed[i], centroid);
        pcl::computeCovarianceMatrix(*zone_decomposed[i], centroid, covariance);
        partial_covariance_2d[i * 3 + 0] = covariance(0, 0);
        partial_covariance_2d[i * 3 + 1] = covariance(0, 1);
        partial_covariance_2d[i * 3 + 2] = covariance(1, 1);
    }
}

void computeHistogramNormalized(pcl::PointCloud<pcl::PointXYZI>::Ptr pc,
                                int horiz_bins, int verti_bins,
                                float *histogram)
{
    Eigen::Vector4f min, max, min_box, max_box;
    pcl::getMinMax3D(*pc, min, max);
    float horiz_itv, verti_itv;
    horiz_itv = (max[0] - min[0] > max[1] - min[1])
                    ? (max[0] - min[0]) / horiz_bins
                    : (max[1] - min[1]) / horiz_bins;
    verti_itv = (max[2] - min[2]) / verti_bins;

    for (int i = 0; i < horiz_bins; i++)
    {
        for (int j = 0; j < verti_bins; j++)
        {
            if (max[0] - min[0] > max[1] - min[1])
            {
                min_box << min[0] + horiz_itv * i, min[1], min[2] + verti_itv * j, 0;
                max_box << min[0] + horiz_itv * (i + 1), max[1],
                    min[2] + verti_itv * (j + 1), 0;
            }
            else
            {
                min_box << min[0], min[1] + horiz_itv * i, min[2] + verti_itv * j, 0;
                max_box << max[0], min[1] + horiz_itv * (i + 1),
                    min[2] + verti_itv * (j + 1), 0;
            }
            std::vector<int> indices;
            pcl::getPointsInBox(*pc, min_box, max_box, indices);
            histogram[i * verti_bins + j] = (float)indices.size() / (float)pc->size();
        }
    }
}

void computeSlice(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, int n,
                  float *slice)
{
    Eigen::Vector4f pc_min, pc_max;
    pcl::getMinMax3D(*pc, pc_min, pc_max);

    pcl::PointCloud<pcl::PointXYZI>::Ptr blocks[n];
    float itv = (pc_max[2] - pc_min[2]) / n;
    if (itv > 0)
    {
        for (int i = 0; i < n; i++)
        {
            blocks[i].reset(new pcl::PointCloud<pcl::PointXYZI>);
        }
        for (unsigned int i = 0, j; i < pc->size(); i++)
        {
            j = std::min((n - 1), (int)((pc->points[i].z - pc_min[2]) / itv));
            blocks[j]->points.push_back(pc->points[i]);
        }

        Eigen::Vector4f block_min, block_max;
        for (int i = 0; i < n; i++)
        {
            if (blocks[i]->size() > 0)
            {
                // pcl::PCA<pcl::PointXYZI> pca;
                // pcl::PointCloud<pcl::PointXYZI>::Ptr block_projected(new
                // pcl::PointCloud<pcl::PointXYZI>); pca.setInputCloud(blocks[i]);
                // pca.project(*blocks[i], *block_projected);
                pcl::getMinMax3D(*blocks[i], block_min, block_max);
            }
            else
            {
                block_min.setZero();
                block_max.setZero();
            }
            slice[i * 2] = block_max[0] - block_min[0];
            slice[i * 2 + 1] = block_max[1] - block_min[1];
        }
    }
    else
    {
        for (int i = 0; i < 20; i++)
            slice[i] = 0;
    }
}

void computeIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, int bins,
                      float *intensity)
{
    float sum = 0, mean = 0, sum_dev = 0;
    float min = FLT_MAX, max = -FLT_MAX;
    for (int i = 0; i < 27; i++)
        intensity[i] = 0;

    for (size_t i = 0; i < pc->size(); i++)
    {
        sum += pc->points[i].intensity;
        min = std::min(min, pc->points[i].intensity);
        max = std::max(max, pc->points[i].intensity);
    }
    mean = sum / pc->size();

    for (size_t i = 0; i < pc->size(); i++)
    {
        sum_dev +=
            (pc->points[i].intensity - mean) * (pc->points[i].intensity - mean);
        int ii =
            std::min(float(bins - 1), std::floor((pc->points[i].intensity - min) /
                                                 ((max - min) / bins)));
        intensity[ii]++;
    }
    intensity[25] = sqrt(sum_dev / pc->size());
    intensity[26] = mean;
}

class ObjectClassifier
{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nhPrivate_;

        ros::Subscriber pc_sub;
        ros::Publisher pub_ttdc_object;
        ros::Publisher pub_bb;
        ros::Publisher marker_array_pub_;
        ros::Publisher pub_pc;

        ros::Publisher autoware_detection_pub;
        ros::Publisher cloud_cluster_pub;

        std::vector<Feature> features_;

        struct svm_node *svm_node_;
        struct svm_model *svm_model_;

        float human_probability_;
        float x_lower_;
        float x_upper_;

        float svm_scale_range_[FEATURE_SIZE][2];
        std::string input_topic_name, model_file_name_, range_file_name_;
        bool use_svm_model_;

        // Clustring Parameters
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud{
            new pcl::PointCloud<pcl::PointXYZI>};

        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;

        std::string input_topic, frame_id_;
        double ratio_depth_tolerance;
        double min_tolerance;
        double max_tolerance;
        int min_cluster_size;
        int max_cluster_size;
        bool human_size_limit_;
        bool is_probability_model_;
        double height_threshold;

    public:
        ObjectClassifier();
        ~ObjectClassifier();

        void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msgs);
        void PublishCloud();
        void PublishCluster(std::vector<size_t> idx, std::map<size_t, geometry_msgs::Polygon> pol);
        geometry_msgs::Point Centroids(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

        void extractFeature(pcl::PointCloud<pcl::PointXYZI>::Ptr pc, Feature &f,
                            Eigen::Vector4f &min, Eigen::Vector4f &max,
                            Eigen::Vector4f &centroid);

        void Clustering(void);
        geometry_msgs::Vector3 calculateSize(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
        double ComputeTolerance(const pcl::PointXYZI &point);
        bool CustomCondition(const pcl::PointXYZI &seed_point,
                            const pcl::PointXYZI &candidate_point,
                            float squared_distance);

        void saveFeature(Feature &f, struct svm_node *x);
        void classify();
};
