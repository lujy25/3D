#include "function.h"
//生成点云显示器
boost::shared_ptr<pcl::visualization::PCLVisualizer> generate_viewer() {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //设置背景颜色
    viewer->setBackgroundColor (255, 255, 255);
    //设置相机视角宽度
    viewer->setCameraFieldOfView(0.99);
    //设置相机位置与朝向
//    viewer->setCameraPosition(
//        0.219005, 1.25186, 1.52278,  // camera位置
//        0.219005, 0, -1,   // view向量--相机朝向
//        0, 0, 0
//    );
    //设置显示器大小
    viewer->setSize (1560, 820);
    return viewer;
}

//等待点云显示窗口关闭
void wait_cloud_show(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer) {
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}


template <typename PointT>
void set_view_point(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer, const typename PointCloud<PointT>::Ptr& view_cloud) {
    //计算点云坐标范围，设置观测视点
    PointT min;//用于存放三个轴的最小值
    PointT max;//用于存放三个轴的最大值

    pcl::getMinMax3D(*view_cloud, min, max);

    viewer->setCameraPosition(
        (min.x + max.x) / 2, max.y + (max.y - min.y) * 0.5, max.z + (max.z - min.z) * 2,  // camera位置
        (min.x + max.x) / 2, 0, -1,  // view向量--相机朝向
        0, 0, 0
    );
    viewer->addCube(min.x, max.x, min.y, max.y, min.z, max.z, 0, 0, 1);
}

void show_cloud(const PointCloud<PointXYZRGB>::Ptr& cloud) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = generate_viewer();

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "cloud");

    //设置观测视点
    set_view_point<pcl::PointXYZRGB>(viewer, cloud);

    wait_cloud_show(viewer);
}
