#include "function.h"

Mat read_dir_img(string root_dir, string img_name) {
    ostringstream file_oss;
    file_oss << root_dir << "/" << img_name << ".png";
    if (img_name == "left" || img_name == "right") {
        return imread(file_oss.str().c_str()); // RGB
    } else {
        assert(img_name == "depth");
        return imread(file_oss.str().c_str(), IMREAD_UNCHANGED);
    }
}

//read camera param
void read_param(string root_dir, float& cx, float& cy, float& fx, float& fy) {
	ostringstream param_oss;
    param_oss << root_dir << "/param.txt";
	ifstream paramRead(param_oss.str().c_str());
    paramRead >> cx  >> cy >> fx >> fy;
}

PointCloud<PointXYZRGB>::Ptr construct_point_cloud(const Mat& rgb, const Mat& depth, const float& cx, const float& cy, const float& fx, const float& fy) {
	// 相机坐标系下的点云
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	for (int row = 0; row < depth.rows; ++row) {
		for (int col = 0; col < depth.cols; ++col) {
			ushort d = depth.ptr<ushort>(row)[col];
			if (d <= 0) continue; //剔除深度值为空的点
			PointXYZRGB p;
			//二维坐标与三维坐标转换关系
			p.z = static_cast<float>(d) / cam_factor;
			p.x = (col - cx) * p.z / fx;
			p.y = (row - cy) * p.z / fy;

            // 为便于显示，绕x轴三维旋转180°
			p.y = -p.y;
			p.z = -p.z;

			// 赋予对应颜色值
			p.b = rgb.ptr<uchar>(row)[col * 3];
			p.g = rgb.ptr<uchar>(row)[col * 3 + 1];
			p.r = rgb.ptr<uchar>(row)[col * 3 + 2];
			cloud->points.push_back(p);
		}
	}
    return cloud;
}

PointCloud<PointXYZRGB>::Ptr filter_cloud(const PointCloud<PointXYZRGB>::Ptr& cloud) {
	//创建滤波器
	PointCloud<PointXYZRGB>::Ptr filtered_cloud(new PointCloud<PointXYZRGB>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlierRemovaler;   //创建滤波器对象
    outlierRemovaler.setInputCloud(cloud);                           //设置待滤波的点云
    outlierRemovaler.setMeanK(50);                               //设置在进行统计时考虑查询点临近点数
    outlierRemovaler.setStddevMulThresh(1.0);                      //设置判断是否为离群点的阀值
    outlierRemovaler.filter(*filtered_cloud);
    return filtered_cloud;
}




PointCloud<PointXYZRGB>::Ptr get_point_cloud(string root_dir) {
    float cx, cy, fx, fy;
    read_param(root_dir, cx, cy, fx, fy);
    Mat rgb = read_dir_img(root_dir, "left");
    Mat depth = read_dir_img(root_dir, "depth");
    PointCloud<PointXYZRGB>::Ptr cloud = construct_point_cloud(rgb, depth, cx, cy, fx, fy);
    return cloud;
}

void construct_sift_point_cloud(const string& source_dir, const string& target_dir, PointCloud<PointXYZ>::Ptr& source_sift_cloud, PointCloud<PointXYZ>::Ptr& target_sift_cloud) {
    Mat source_depth = read_dir_img(source_dir, "depth");
    Mat target_depth = read_dir_img(target_dir, "depth");
    float cx, cy, fx, fy;
    ostringstream param_oss;
    param_oss << source_dir << "/param.txt";
	ifstream paramRead(param_oss.str().c_str());
    paramRead >> cx  >> cy >> fx >> fy;

    ostringstream source_sift_oss;
    source_sift_oss << "build/" << source_dir << "/sift.txt";
	ifstream sourceSiftRead(source_sift_oss.str().c_str());

	ostringstream target_sift_oss;
    target_sift_oss << "build/" << target_dir <<  "/sift.txt";
	ifstream targetSiftRead(target_sift_oss.str().c_str());

	int source_row, source_col, target_row, target_col;
	ushort source_d, target_d;
	while (sourceSiftRead >> source_col >> source_row) {
	    if (sourceSiftRead.eof()) break;
	    targetSiftRead >> target_col >> target_row;
        source_d = source_depth.ptr<ushort>(source_row)[source_col];
        target_d = target_depth.ptr<ushort>(target_row)[target_col];
        if (source_d == 0 || target_d == 0) continue;
        PointXYZ source_p, target_p;

        // source sift point
        source_p.z = static_cast<float>(source_d) / cam_factor;
        source_p.x = (source_col - cx) * source_p.z / fx;
        source_p.y = (source_row - cy) * source_p.z / fy;
        // 为便于显示，绕x轴三维旋转180°
        source_p.y = -source_p.y;
        source_p.z = -source_p.z;

        source_sift_cloud->points.push_back(source_p);

        // target sift point
        target_p.z = static_cast<float>(target_d) / cam_factor;
        target_p.x = (target_col - cx) * target_p.z / fx;
        target_p.y = (target_row - cy) * target_p.z / fy;
        // 为便于显示，绕x轴三维旋转180°
        target_p.y = -target_p.y;
        target_p.z = -target_p.z;

        target_sift_cloud->points.push_back(target_p);
	}
}