#ifndef FUNCTION_H
#define FUNCTION_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <iostream>
#include <fstream>
using namespace cv;
using namespace std;
using namespace pcl;
const double cam_factor = 1000;
Mat get_filter_depth(string root_dir);
PointCloud<PointXYZRGB>::Ptr get_point_cloud(string root_dir);
void construct_sift_point_cloud(const string& source_dir, const string& target_dir, PointCloud<PointXYZ>::Ptr& source_sift_cloud, PointCloud<PointXYZ>::Ptr& target_sift_cloud);
PointCloud<PointXYZRGB>::Ptr filter_cloud(const PointCloud<PointXYZRGB>::Ptr& cloud);
Eigen::Matrix4f calculate_stitch_transformation(const PointCloud<PointXYZ>::Ptr& source_cloud, const PointCloud<PointXYZ>::Ptr& target_cloud);
void show_cloud(const PointCloud<PointXYZRGB>::Ptr& cloud);
#endif