#include "util/function.h"



int main (int argc, char** argv) {
    float cx, cy, fx, fy;
    string source_dir = "dataset2/save_0";
    string target_dir = "dataset2/save_1";

    PointCloud<PointXYZRGB>::Ptr source_cloud = get_point_cloud(source_dir);
    PointCloud<PointXYZRGB>::Ptr target_cloud = get_point_cloud(target_dir);

    PointCloud<PointXYZ>::Ptr source_sift_cloud(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr target_sift_cloud(new PointCloud<PointXYZ>);
    construct_sift_point_cloud(source_dir, target_dir, source_sift_cloud, target_sift_cloud);

    Eigen::Matrix4f transform = calculate_stitch_transformation(source_sift_cloud, target_sift_cloud);
    transformPointCloud(*source_cloud, *source_cloud, transform);
    *source_cloud += *target_cloud;
    PointCloud<PointXYZRGB>::Ptr filtered_cloud = filter_cloud(source_cloud);
    show_cloud(filtered_cloud);
    return 0;
}